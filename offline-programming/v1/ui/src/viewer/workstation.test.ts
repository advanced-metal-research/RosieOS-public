import assert from "node:assert/strict";
import test from "node:test";

import { adaptCadQueryTopology } from "../adapters/cadquery.ts";
import { createWeldsFromRecommendations } from "../adapters/weld-authoring.ts";
import type { CadTopology } from "../cad/contracts.ts";
import { previewTrajectory } from "../domain/playback.ts";
import { plannerPoseToWorld } from "../domain/placement.ts";
import { createProject, installPreviewPlan } from "../domain/project.ts";
import type { OfflineProgrammingProject } from "../domain/project.ts";
import {
  createBlankSpreadsheetProject,
  createSpreadsheetRow,
} from "../domain/spreadsheet.ts";
import { createDogBoxProject } from "../examples/dog-box.ts";
import {
  projectCurveCenters,
  projectPath,
  projectRowLabels,
  projectSelectionOverlay,
  projectTcpFrames,
  toSimulationSample,
  syncProjectToWorkstation,
} from "./workstation.ts";
import type { WorkstationRenderer } from "./workstation-renderer.ts";
import {
  attachMoveMetadata,
  buildProgramView,
  interpolateThreePointArc,
} from "../domain/points-moves.ts";

test("authoring updates overlays without changing the robot pose or live feedback state", async () => {
  const touched: string[] = [];
  const renderer = {
    loadModel: async () => ({ ok: true, modelId: "rosie_1400_v3", assetErrors: [] }),
    setOffline: () => assert.fail("authoring must not disconnect the live pose"),
    setPose: () => assert.fail("authoring must not reposition the robot"),
    ...Object.fromEntries(["setSeams", "setWorkpiece", "setWorkcellFixtures", "setPath", "setPathDecorations", "setSelectionOverlay", "setTcpFrames", "setCurveCenters", "setCollisionMarkers"].map(name => [name, () => touched.push(name)])),
  } as unknown as WorkstationRenderer;
  const result = await syncProjectToWorkstation(renderer, createProject());
  assert.equal(result.ok, true);
  assert.ok(touched.includes("setPath"));
});

test("project adapter visualizes the loaded spreadsheet rather than a stale plan or CAD overlay", () => {
  const authored = createDogBoxProject("2026-07-30T00:00:00.000Z");
  const project = installPreviewPlan(authored, previewTrajectory(authored.rows));
  const enabledMotionRows = project.rows.filter((row) =>
    row.enabled && (row.rowType ?? "motion") === "motion"
  );
  const changedTarget = [0.91, -0.27, 0.63, 0, 0, 0] as const;
  project.rows[0]!.targetTcpPose = [...changedTarget];
  const path = projectPath(project);
  const frames = projectTcpFrames(project);
  assert.equal(path.length, enabledMotionRows.length);
  assert.deepEqual(path[0]!.positionM, [1.6974, 0.41579999999999995, 1.1108375]);
  assert.notDeepEqual(
    path[0]!.positionM,
    project.plan.trajectory[0]!.tcpPose.slice(0, 3),
  );
  assert.ok(path.some(({ motion }) => motion === "weld"));
  assert.ok(frames.length > 0);
  assert.ok(frames.length <= 25);
  assert.ok(frames.length <= buildProgramView(project).visibleRowIds.length);

  const sample = toSimulationSample(project, project.plan.trajectory[0]!);
  assert.equal(sample.modelId, project.robot.model);
  assert.deepEqual(sample.jointOrder, ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]);
  assert.equal(sample.jointsRad.length, 9);
});

test("curve center handles follow compact move selection rather than sample rows", () => {
  // A CURVE carrying interior sample rows must contribute exactly one handle,
  // keyed by move id and selected through the move's destination CP — not one
  // marker per hidden sample.
  const project = createBlankSpreadsheetProject({ name: "Curve handles" });
  project.rows = Array.from({ length: 7 }, (_, index) => {
    const row = createSpreadsheetRow(project, "motion", `curve-${index}`);
    row.targetTcpPose = [1.2 + index / 60, 0, 0.8, 0, 0, 0];
    return row;
  });
  attachMoveMetadata(project.rows, [{
    startIndex: 0,
    endIndex: 6,
    kind: "CURVE",
    centerM: [1.25, 0.05, 0.8],
  }]);

  const curve = buildProgramView(project).moves.find((move) => move.kind === "CURVE");
  assert.ok(curve?.centerM);
  assert.equal(curve.sampleRows.length, 5);

  project.simulation.selectedRowId = curve.end.row.id;
  const centers = projectCurveCenters(project);
  assert.deepEqual(centers.map((center) => center.id), [curve.id]);
  assert.equal(centers[0]!.selected, true);
  assert.deepEqual(
    centers[0]!.positionM,
    plannerPoseToWorld(project, [...curve.centerM, 0, 0, 0]).slice(0, 3),
  );

  // Selecting a hidden sample row is not the move's destination CP: the handle
  // stays visible but unselected.
  project.simulation.selectedRowId = curve.sampleRows[2]!.id;
  const unselected = projectCurveCenters(project);
  assert.deepEqual(unselected.map((center) => center.id), [curve.id]);
  assert.equal(unselected[0]!.selected, false);
});

test("CAD-imported circle seams seed curve handles from arc geometry", () => {
  const project = createCircleCadProject();
  const view = buildProgramView(project);
  const curves = view.moves.filter((move) => move.kind === "CURVE" && move.centerM);
  assert.ok(curves.length >= 2);
  assert.equal(projectCurveCenters(project).length, curves.length);
  assert.deepEqual(
    projectCurveCenters(project).map((center) => center.id).sort(),
    curves.map((move) => move.id).sort(),
  );
});

test("spreadsheet path follows enabled motion rows in their displayed order", () => {
  const project = createBlankSpreadsheetProject({ name: "Visible rows" });
  project.rows.push(createSpreadsheetRow(project, "motion"));
  const first = project.rows[0]!;
  first.targetTcpPose = [1.0, 0.2, 0.7, 0, 0, 0];
  first.weldEnabled = false;

  const last = createSpreadsheetRow(project, "motion", "visible-last");
  last.targetTcpPose = [0.7, -0.1, 0.82, 0, 0, 0];
  last.weldEnabled = true;

  const comment = createSpreadsheetRow(project, "comment", "ignored-comment");
  comment.targetTcpPose = [9, 9, 9, 0, 0, 0];
  const disabled = createSpreadsheetRow(project, "motion", "ignored-disabled");
  disabled.enabled = false;
  disabled.targetTcpPose = [8, 8, 8, 0, 0, 0];
  project.rows = [last, comment, disabled, first];

  assert.deepEqual(projectPath(project), [
    { positionM: [1.4874, 0.5858, 1.3008374999999999], motion: "weld" },
    { positionM: [1.7873999999999999, 0.8857999999999999, 1.1808375], motion: "travel" },
  ]);
  // Focus mode: a selected motion row keeps only its own TCP frame.
  project.simulation.selectedRowId = first.id;
  assert.deepEqual(
    projectTcpFrames(project).map((frame) => frame.id),
    [`tcp-row-${first.id}`],
  );
  // Without a selection the frames follow the displayed enabled motion order.
  project.simulation.selectedRowId = null;
  assert.deepEqual(
    projectTcpFrames(project).map((frame) => frame.id),
    ["tcp-row-visible-last", `tcp-row-${first.id}`],
  );
});

/** Single circle-like edge imported through the real CadQuery adapter path. */
function createCircleCadProject(): OfflineProgrammingProject {
  const samplesM = interpolateThreePointArc(
    [[0, 0, 0], [0.05, 0.05, 0], [0.1, 0, 0]],
    64,
  ) as [number, number, number][];
  const topology: CadTopology = {
    schemaVersion: 1,
    unit: "m",
    sourceUnit: "mm",
    tool: "test",
    modelId: "curve-handle-cad",
    filename: "curve.step",
    fingerprint: "c".repeat(64),
    sampleCount: samplesM.length,
    parts: [{ id: "part", edgeCount: 1 }],
    edges: [{
      id: "circle",
      label: "ARC",
      partId: "part",
      geometryType: "CIRCLE",
      sampleCount: samplesM.length,
      samplesM,
      lengthM: 0.157,
      boundsMinM: [0, 0, 0],
      boundsMaxM: [0.1, 0.05, 0],
      closed: false,
      sourceEdgeIds: [],
      sourceGeometryTypes: [],
      pathClass: "circle",
      isSeamCandidate: true,
    }],
    rawEdgeCount: 1,
    compositeEdgeCount: 0,
    seamCandidateCount: 1,
    recommendedSeams: [{
      edgeId: "circle",
      partId: "part",
      geometryType: "CIRCLE",
      pathClass: "circle",
      window: { start: 0, end: 1 },
      reversed: false,
      reason: "test",
    }],
  };
  return createWeldsFromRecommendations(
    adaptCadQueryTopology(createProject(), topology),
  );
}


test("taught joint targets and circular via poses render before planning in world coordinates", () => {
  const project = createDogBoxProject("2026-09-25T00:00:00Z");
  const target = { frame_id: "world" as const, xyz_m: [1.2, .3, .9] as [number, number, number],
    rpy_rad: [.1, .2, .3] as [number, number, number], joint_names: ["J1"], joint_values_rad: [0] };
  const node: import("../contracts/robot-v4-program-v2.ts").MoveNode = {
    id: "p1", op: "move", enabled: true, label: "P1", comment: "", row_hash: "p1",
    motion: "joint", target, speed_scale: .1, speed_mm_s: 25,
    capture: { source: "preview", observed_at: "2026-09-25T00:00:00Z",
      model_id: project.robot.model, robot_description_sha256: "test", pose: target },
  };
  const row = createSpreadsheetRow(project, "motion", "p1:target");
  row.moveType = "JOINT";
  row.extensions = { node_id: node.id };
  project.nodes = [node];
  project.rows = [row];
  project.simulation.selectedRowId = row.id;
  assert.deepEqual(projectPath(project).map(p => p.positionM), [target.xyz_m]);
  assert.deepEqual(projectTcpFrames(project)[0]?.pose, [...target.xyz_m, ...target.rpy_rad]);
  assert.equal(projectRowLabels(project)[0]?.text, "P1");
  assert.deepEqual(projectSelectionOverlay(project)?.targetM, target.xyz_m);

  node.motion = "circular";
  node.via = { ...target, xyz_m: [1.3, .4, .9] };
  assert.deepEqual(projectPath(project).map(p => p.positionM), [node.via.xyz_m, target.xyz_m]);
  assert.deepEqual(projectRowLabels(project).map(p => p.text), ["P1 via", "P1"]);
  assert.deepEqual(projectSelectionOverlay(project)?.targetM, target.xyz_m);
  node.enabled = false;
  assert.deepEqual(projectPath(project), []);
});
