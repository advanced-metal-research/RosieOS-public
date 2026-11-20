import assert from "node:assert/strict";
import test from "node:test";
import { Euler, Vector3 } from "three";

import { workpieceTopology } from "../../test/support/workpiece-fixtures.ts";
import { adaptCadQueryTopology, appendCadQueryTopology } from "../adapters/cadquery.ts";
import { createWeldsFromRecommendations } from "../adapters/weld-authoring.ts";
import type { CadWorkpieceMesh } from "../cad/contracts.ts";
import { createProject } from "./project.ts";
import { setWorkpiecePlacement, transformWorkpiecePoint } from "./placement.ts";
import { exportProjectJSON, importProjectJSON } from "./serialization.ts";
import { WorkcellFixtureUnavailable, workcellFixturesWire } from "./workcell.ts";
import {
  activeWorkpieceId, activateWorkpiece, environmentFixtures, forgetActiveWorkpiece,
  projectWorkpieces,
} from "./workpieces.ts";

function placedWeldPart() {
  const project = createWeldsFromRecommendations(adaptCadQueryTopology(
    createProject({ robotModel: "rosie_1420_v1" }), workpieceTopology(),
  ));
  setWorkpiecePlacement(project, {
    offsetFromWorkFrameM: [1.2, -0.4, 0.8], rotationRpyRad: [0.2, -0.3, 0.7],
  });
  return project;
}

test("adding another STEP preserves the previous part's geometry, placement and authored welds", () => {
  const first = placedWeldPart();
  const before = structuredClone(first);
  const original = projectWorkpieces(first)[0]!;
  assert.ok(original.nodes.length > 0);
  assert.ok(original.rows.length > 0);
  const added = appendCadQueryTopology(first, workpieceTopology("second.step", "b".repeat(64)));
  assert.deepEqual(first, before, "adding a part must not mutate undo history");
  const parts = projectWorkpieces(added);
  assert.equal(parts.length, 2);
  assert.deepEqual(parts[0], original);
  assert.equal(parts[1]!.cad.filename, "second.step");
  assert.deepEqual(parts[1]!.nodes, []);
  assert.deepEqual(parts[1]!.seams, []);
  assert.equal(activeWorkpieceId(added), parts[1]!.id);
  assert.deepEqual(added.workcellFixtures, before.workcellFixtures);
});

test("repeated imports of the same filename and content remain separate parts", () => {
  let project = createProject();
  for (let index = 0; index < 4; index++) project = appendCadQueryTopology(project, workpieceTopology());
  const parts = projectWorkpieces(project);
  assert.equal(parts.length, 4);
  assert.equal(new Set(parts.map(part => part.id)).size, 4);
  assert.ok(parts.every(part => part.cad.filename === "bracket.step"));
});

test("switching parts retains edits made since the last switch without cross-part aliasing", () => {
  const project = appendCadQueryTopology(placedWeldPart(), workpieceTopology("second.step", "b".repeat(64)));
  const [first, second] = projectWorkpieces(project);
  project.fixtures[0]!.translationM = [3, 4, 5];
  assert.equal(activateWorkpiece(project, first!.id), true);
  project.nodes[0]!.label = "Edited first part";
  project.fixtures[0]!.translationM = [6, 7, 8];
  assert.equal(activateWorkpiece(project, second!.id), true);
  assert.deepEqual(project.fixtures[0]!.translationM, [3, 4, 5]);
  assert.equal(project.nodes.length, 0);
  assert.equal(activateWorkpiece(project, first!.id), true);
  assert.equal(project.nodes[0]!.label, "Edited first part");
  assert.deepEqual(project.fixtures[0]!.translationM, [6, 7, 8]);
  const before = structuredClone(project);
  assert.equal(activateWorkpiece(project, "missing"), false);
  assert.deepEqual(project, before);
});

test("removing the active part and clearing its working copy leaves the other part available", () => {
  const original = placedWeldPart();
  const survivor = projectWorkpieces(original)[0]!;
  const project = appendCadQueryTopology(original, workpieceTopology("remove.step", "b".repeat(64)));
  forgetActiveWorkpiece(project);
  project.edgeCatalog = [];
  project.fixtures = [];
  project.seams = [];
  project.nodes = [];
  project.rows = [];
  project.sequence = [];
  project.cad = { kind: "step", filename: "", fingerprint: "", sourceUnit: "mm", topologyModelId: "" };
  assert.deepEqual(projectWorkpieces(project), [survivor]);
  assert.equal(activateWorkpiece(project, survivor.id), true);
  assert.deepEqual(projectWorkpieces(project), [survivor]);
});

test("save/load keeps all parts and the latest active edits", () => {
  const project = appendCadQueryTopology(placedWeldPart(), workpieceTopology("second.step", "b".repeat(64)));
  project.fixtures[0]!.translationM = [-1, 2, 3];
  const restored = importProjectJSON(exportProjectJSON(project));
  assert.equal(activeWorkpieceId(restored), activeWorkpieceId(project));
  assert.deepEqual(projectWorkpieces(restored), projectWorkpieces(project));
  const first = projectWorkpieces(restored)[0]!;
  assert.equal(activateWorkpiece(restored, first.id), true);
  assert.deepEqual(restored.nodes, first.nodes);
  assert.deepEqual(restored.fixtures, first.fixtures);
});

test("inactive parts join the stand as collision geometry with their full CAD-to-world rotation", () => {
  const first = placedWeldPart();
  const firstId = activeWorkpieceId(first);
  const project = appendCadQueryTopology(first, workpieceTopology("second.step", "b".repeat(64)));
  const fixtures = environmentFixtures(project);
  assert.equal(fixtures.length, 2);
  assert.equal(fixtures[0]!.id, "stand");
  const obstacle = fixtures.find(fixture => fixture.id === `workpiece:${firstId}`)!;
  assert.deepEqual(obstacle.shape, { kind: "step", fingerprint: first.cad.fingerprint, filename: first.cad.filename });
  const localPoint: [number, number, number] = [0.15, 0.07, 0.02];
  const placed = new Vector3(...localPoint)
    .applyEuler(new Euler(...obstacle.rotationRpyRad, "ZYX"))
    .add(new Vector3(...obstacle.translationM)).toArray();
  const expected = transformWorkpiecePoint(first, localPoint);
  placed.forEach((coordinate, index) => assert.ok(Math.abs(coordinate - expected[index]!) < 1e-10));
  assert.throws(() => workcellFixturesWire(fixtures, new Map()), WorkcellFixtureUnavailable);
  const mesh: CadWorkpieceMesh = {
    schemaVersion: 1, unit: "m", sourceUnit: "mm", tool: "test", modelId: "test",
    filename: first.cad.filename, fingerprint: first.cad.fingerprint,
    positionsM: new Float32Array([0, 0, 0, 0.2, 0, 0, 0, 0.2, 0]), indices: new Uint32Array([0, 1, 2]),
    normals: null, vertexCount: 3, triangleCount: 1, boundsMinM: [0, 0, 0], boundsMaxM: [0.2, 0.2, 0],
    linearDeflectionM: 0.001, angularDeflectionRad: 0.1, solidCount: 1, truncated: false,
  };
  const wire = workcellFixturesWire(fixtures, new Map([[mesh.fingerprint, mesh]]));
  assert.equal(wire[1]!.shape.kind, "mesh");
  assert.deepEqual(wire[1]!.xyz_m, obstacle.translationM);
  assert.deepEqual(wire[1]!.rpy_rad, obstacle.rotationRpyRad);
  const secondId = activeWorkpieceId(project);
  assert.equal(activateWorkpiece(project, firstId), true);
  assert.deepEqual(environmentFixtures(project).map(fixture => fixture.id), ["stand", `workpiece:${secondId}`]);
});
