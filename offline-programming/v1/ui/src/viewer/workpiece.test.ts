import assert from "node:assert/strict";
import test from "node:test";
import { BufferGeometry, Group, Mesh, Vector3 } from "three";

import type { CadWorkpieceMesh } from "../cad/contracts.ts";
import { createDogBoxProject } from "../examples/dog-box.ts";
import {
  centreWorkpieceOnWorkFrame,
  setWorkpiecePlacement,
  transformWorkpiecePoint,
} from "../domain/placement.ts";
import {
  applyWorkpiecePlacement,
  createWorkpieceMeshGroup,
  workpieceMeshIdentity,
  workpieceMeshSummary,
} from "./workpiece.ts";
import {
  projectWorkpiecePlacement,
  workpieceMeshForProject,
} from "./workstation.ts";

const FINGERPRINT = "b".repeat(64);

/** Typed access to a built layer child, so assertions read the real geometry. */
function meshChild(group: Group, index: number): Mesh<BufferGeometry> {
  const child = group.children[index];
  assert.ok(child instanceof Mesh, `child ${index} is not a Mesh`);
  return child as Mesh<BufferGeometry>;
}

/** A unit tetrahedron in metres, in CAD-local coordinates. */
function testMesh(overrides: Partial<CadWorkpieceMesh> = {}): CadWorkpieceMesh {
  return {
    schemaVersion: 1,
    unit: "m",
    sourceUnit: "mm",
    tool: "rosie cadquery v1 workpiece mesh",
    modelId: "cadquery-step-bbbbbbbbbbbbbbbb",
    filename: "part.step",
    fingerprint: FINGERPRINT,
    positionsM: new Float32Array([
      0, 0, 0,
      0.1, 0, 0,
      0, 0.1, 0,
      0, 0, 0.1,
    ]),
    indices: new Uint32Array([0, 1, 2, 0, 1, 3, 0, 2, 3, 1, 2, 3]),
    normals: null,
    vertexCount: 4,
    triangleCount: 4,
    boundsMinM: [0, 0, 0],
    boundsMaxM: [0.1, 0.1, 0.1],
    linearDeflectionM: 0.0005,
    angularDeflectionRad: 0.5,
    solidCount: 1,
    truncated: false,
    ...overrides,
  };
}

test("workpiece layer builds an indexed surface and computes missing normals", () => {
  const group = createWorkpieceMeshGroup(testMesh(), null);
  assert.equal(group.name, "workstation-workpiece");
  assert.equal(group.children.length, 1);
  const surface = meshChild(group, 0);
  assert.equal(surface.type, "Mesh");
  assert.equal(surface.geometry.index?.count, 12);
  assert.equal(surface.geometry.getAttribute("position")?.count, 4);
  // No normals on the wire, so the viewer must derive them rather than render
  // an unlit black part.
  assert.equal(surface.geometry.getAttribute("normal")?.count, 4);
  assert.equal(group.userData.triangleCount, 4);
  assert.equal(group.userData.fingerprint, FINGERPRINT);
});

test("supplied normals are used verbatim instead of being recomputed", () => {
  const normals = new Float32Array([
    0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1,
  ]);
  const group = createWorkpieceMeshGroup(testMesh({ normals }), null);
  assert.deepEqual(
    Array.from(meshChild(group, 0).geometry.getAttribute("normal")?.array ?? []),
    Array.from(normals),
  );
});

test("an absent or empty mesh yields an empty but valid layer", () => {
  for (
    const mesh of [
      null,
      testMesh({ indices: new Uint32Array(), triangleCount: 0 }),
      testMesh({ positionsM: new Float32Array(), vertexCount: 0 }),
    ]
  ) {
    const group = createWorkpieceMeshGroup(mesh, null);
    assert.equal(group.name, "workstation-workpiece");
    assert.equal(group.children.length, 0, "no-mesh layer must be empty, not partial");
  }
});

test("the optional wireframe is off by default and additive when enabled", () => {
  assert.equal(createWorkpieceMeshGroup(testMesh(), null).children.length, 1);
  const edged = createWorkpieceMeshGroup(testMesh(), null, { showEdges: true });
  assert.equal(edged.children.length, 2);
  assert.equal(edged.children[1]?.name, "workstation-workpiece-edges");
});

test("the mesh layer transform reproduces the seam placement transform exactly", () => {
  const project = createDogBoxProject("2026-08-04T00:00:00.000Z");
  // A compound rotation, not a pure yaw: every Euler order agrees on a yaw,
  // and the mesh had been reading the "ZYX" components back as "XYZ".
  setWorkpiecePlacement(project, {
    offsetFromWorkFrameM: [0.12, -0.04, 0.03],
    rotationRpyRad: [0.4, -0.7, 1.1],
  });
  const placement = projectWorkpiecePlacement(project);
  assert.ok(placement);
  const group = createWorkpieceMeshGroup(testMesh(), placement);
  group.updateMatrixWorld(true);

  // Every CAD-local vertex must land where transformWorkpiecePoint puts it —
  // that is the function the weld path overlays already use.
  for (const local of [[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0], [0.05, -0.02, 0.11]]) {
    const expected = transformWorkpiecePoint(
      project,
      local as [number, number, number],
    );
    const actual = group.localToWorld(
      new Vector3(local[0]!, local[1]!, local[2]!),
    );
    for (const [axis, value] of [expected[0], expected[1], expected[2]].entries()) {
      const got = [actual.x, actual.y, actual.z][axis]!;
      assert.ok(
        Math.abs(got - value) < 1e-9,
        `axis ${axis}: mesh ${got} vs seam transform ${value}`,
      );
    }
  }
});

test("placement edits and CENTRE move the existing layer without rebuilding it", () => {
  const project = createDogBoxProject("2026-08-04T00:00:00.000Z");
  setWorkpiecePlacement(project, {
    offsetFromWorkFrameM: [0.2, 0.1, 0],
    rotationRpyRad: [0, 0, 0],
  });
  const group = createWorkpieceMeshGroup(testMesh(), projectWorkpiecePlacement(project));
  const geometryBefore = meshChild(group, 0).geometry;
  const movedFrom = group.position.clone();

  setWorkpiecePlacement(project, {
    offsetFromWorkFrameM: [-0.15, 0.05, 0.02],
    rotationRpyRad: [0, 0, Math.PI / 2],
  });
  applyWorkpiecePlacement(group, projectWorkpiecePlacement(project));
  assert.notDeepEqual(group.position.toArray(), movedFrom.toArray());
  assert.ok(Math.abs(group.rotation.z - Math.PI / 2) < 1e-9);
  assert.equal(
    meshChild(group, 0).geometry,
    geometryBefore,
    "a placement edit must not rebuild the mesh geometry",
  );

  centreWorkpieceOnWorkFrame(project);
  const centred = projectWorkpiecePlacement(project);
  assert.ok(centred);
  applyWorkpiecePlacement(group, centred);
  assert.deepEqual(
    centred.originM.map((value) => Math.round(value * 1e6) / 1e6),
    transformWorkpiecePoint(project, [0, 0, 0]).map((value) =>
      Math.round(value * 1e6) / 1e6
    ),
  );
  assert.deepEqual(group.rotation.toArray().slice(0, 3), [0, 0, 0]);
});

test("a missing placement leaves the layer at the world origin rather than nowhere", () => {
  const group = createWorkpieceMeshGroup(testMesh(), null);
  group.position.set(1, 2, 3);
  applyWorkpiecePlacement(group, null);
  assert.deepEqual(group.position.toArray(), [0, 0, 0]);
  assert.deepEqual(group.rotation.toArray().slice(0, 3), [0, 0, 0]);
});

test("non-finite placement values are neutralized instead of poisoning the matrix", () => {
  const group = createWorkpieceMeshGroup(testMesh(), {
    originM: [Number.NaN, 1, Number.POSITIVE_INFINITY],
    rotationRpyRad: [Number.NaN, 0, 0],
  });
  assert.deepEqual(group.position.toArray(), [0, 1, 0]);
  assert.equal(group.rotation.x, 0);
});

test("a mesh only applies to the project whose CAD fingerprint it matches", () => {
  const project = createDogBoxProject("2026-08-04T00:00:00.000Z");
  const mesh = testMesh();
  // The browser-fallback example carries "example:dog-box-v1", never a digest.
  assert.equal(workpieceMeshForProject(project, mesh), null);
  assert.equal(workpieceMeshForProject(project, null), null);

  project.cad.fingerprint = FINGERPRINT.toUpperCase();
  assert.equal(workpieceMeshForProject(project, mesh), mesh);
  assert.equal(workpieceMeshForProject(project, testMesh({ fingerprint: "c".repeat(64) })), null);
});

test("mesh identity changes only when the geometry or display options change", () => {
  const mesh = testMesh();
  assert.equal(workpieceMeshIdentity(mesh), workpieceMeshIdentity(testMesh()));
  assert.notEqual(workpieceMeshIdentity(mesh), workpieceMeshIdentity(mesh, { showEdges: true }));
  assert.notEqual(
    workpieceMeshIdentity(mesh),
    workpieceMeshIdentity(testMesh({ triangleCount: 8 })),
  );
  assert.notEqual(
    workpieceMeshIdentity(mesh),
    workpieceMeshIdentity(testMesh({ fingerprint: "c".repeat(64) })),
  );
});

test("the status summary reports triangles, extent, and any truncation", () => {
  assert.equal(
    workpieceMeshSummary(testMesh()),
    "Workpiece mesh · 4 triangles · 100 × 100 × 100 mm",
  );
  assert.match(workpieceMeshSummary(testMesh({ truncated: true })), /truncated at the triangle cap$/);
  assert.equal(
    workpieceMeshSummary(testMesh({ boundsMinM: [0, 0, 0], boundsMaxM: [0, 0, 0] })),
    "Workpiece mesh · 4 triangles",
  );
});
