import * as THREE from "three";

import type { CadWorkpieceMesh } from "../cad/contracts.ts";
import type { WorkcellFixture } from "../domain/workcell.ts";

/**
 * The workcell fixtures drawn in the viewport: the stand the part sits on,
 * as the box or the STEP the operator defined, at its pose in the cell's
 * world frame. Grey and matte like the pedestal, with its edges drawn, so a
 * stand reads as structure rather than as a part. The same list goes to the
 * weld planner as obstacles (`domain/workcell.ts`), so what is drawn here is
 * what the planner clears.
 */
export const WORKCELL_FIXTURE_LAYER_NAME = "workstation-fixtures";

const FIXTURE_COLOR = 0xb3b9bf;
const FIXTURE_EDGE_COLOR = 0x5f6670;

export function createWorkcellFixtureGroup(
  fixtures: readonly WorkcellFixture[],
  meshes: ReadonlyMap<string, CadWorkpieceMesh>,
): THREE.Group {
  const group = new THREE.Group();
  group.name = WORKCELL_FIXTURE_LAYER_NAME;
  for (const fixture of fixtures) {
    const geometry = fixtureGeometry(fixture, meshes);
    if (!geometry) continue;
    const material = new THREE.MeshStandardMaterial({
      color: FIXTURE_COLOR,
      metalness: 0.15,
      roughness: 0.75,
      side: THREE.DoubleSide,
      polygonOffset: true,
      polygonOffsetFactor: 1,
      polygonOffsetUnits: 1,
    });
    const body = new THREE.Mesh(geometry, material);
    body.name = `workcell-fixture:${fixture.id}`;
    body.castShadow = false;
    body.receiveShadow = false;
    const edges = new THREE.LineSegments(
      new THREE.EdgesGeometry(geometry, 20),
      new THREE.LineBasicMaterial({ color: FIXTURE_EDGE_COLOR, transparent: true, opacity: 0.6 }),
    );
    edges.name = `workcell-fixture-edges:${fixture.id}`;
    const placed = new THREE.Group();
    placed.name = `workcell-fixture-placed:${fixture.id}`;
    placed.userData.fixtureId = fixture.id;
    placed.add(body, edges);
    const [x, y, z] = fixture.translationM;
    const [roll, pitch, yaw] = fixture.rotationRpyRad;
    placed.position.set(x, y, z);
    // "ZYX": fixed-axis roll, pitch, yaw, the convention every placement here uses.
    placed.rotation.set(roll, pitch, yaw, "ZYX");
    group.add(placed);
  }
  return group;
}

/** A box stands on its bottom-centre; a STEP is placed by its own origin. */
function fixtureGeometry(
  fixture: WorkcellFixture,
  meshes: ReadonlyMap<string, CadWorkpieceMesh>,
): THREE.BufferGeometry | null {
  if (fixture.shape.kind === "box") {
    const [sx, sy, sz] = fixture.shape.sizeM;
    const box = new THREE.BoxGeometry(sx, sy, sz);
    box.translate(0, 0, sz / 2);
    return box;
  }
  const mesh = meshes.get(fixture.shape.fingerprint.toLowerCase());
  if (!mesh || mesh.indices.length === 0 || mesh.positionsM.length === 0) return null;
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(mesh.positionsM, 3));
  geometry.setIndex(new THREE.BufferAttribute(mesh.indices, 1));
  if (mesh.normals && mesh.normals.length === mesh.positionsM.length) {
    geometry.setAttribute("normal", new THREE.BufferAttribute(mesh.normals, 3));
  } else {
    geometry.computeVertexNormals();
  }
  geometry.computeBoundingSphere();
  return geometry;
}
