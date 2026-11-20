import assert from "node:assert/strict";
import test from "node:test";

import { workpieceTopology } from "../../test/support/workpiece-fixtures.ts";
import { appendCadQueryTopology } from "../adapters/cadquery.ts";
import type { CadWorkpieceMesh } from "../cad/contracts.ts";
import { createProject } from "../domain/project.ts";
import { activeWorkpieceId, activateWorkpiece } from "../domain/workpieces.ts";
import type { WorkcellFixture } from "../domain/workcell.ts";
import { syncProjectToWorkstation } from "./workstation.ts";
import type { WorkstationRenderer } from "./workstation-renderer.ts";

test("the renderer keeps inactive imported parts alongside the stand when the active part changes", async () => {
  let project = appendCadQueryTopology(createProject({ robotModel: "rosie_1420_v1" }), workpieceTopology());
  const firstId = activeWorkpieceId(project);
  project = appendCadQueryTopology(project, workpieceTopology("second.step", "b".repeat(64)));
  const secondId = activeWorkpieceId(project);
  const meshes = new Map<string, CadWorkpieceMesh>();
  const drawn: WorkcellFixture[][] = [];
  const renderer = {
    loadModel: async () => ({ ok: true }),
    setWorkcellFixtures: (fixtures: WorkcellFixture[], suppliedMeshes: ReadonlyMap<string, CadWorkpieceMesh>) => {
      drawn.push(fixtures);
      assert.equal(suppliedMeshes, meshes);
    },
    setOffline: () => {}, setSeams: () => {}, setWorkpiece: () => {}, setPath: () => {},
    setPathDecorations: () => {}, setSelectionOverlay: () => {}, setTcpFrames: () => {},
    setCurveCenters: () => {}, setCollisionMarkers: () => {},
  } as unknown as WorkstationRenderer;
  await syncProjectToWorkstation(renderer, project, { fixtureMeshes: meshes });
  assert.deepEqual(drawn[0]!.map(fixture => fixture.id), ["stand", `workpiece:${firstId}`]);
  activateWorkpiece(project, firstId);
  await syncProjectToWorkstation(renderer, project, { fixtureMeshes: meshes });
  assert.deepEqual(drawn[1]!.map(fixture => fixture.id), ["stand", `workpiece:${secondId}`]);
  assert.equal(drawn[1]![1]!.name, "second.step");
});
