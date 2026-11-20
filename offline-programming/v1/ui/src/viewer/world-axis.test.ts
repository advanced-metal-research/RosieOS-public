import assert from "node:assert/strict";
import test from "node:test";
import * as THREE from "three";

import { projectWorldAxes } from "./world-axis.ts";

test("world axes project into the default camera screen orientation", () => {
  const axes = projectWorldAxes(new THREE.Quaternion());
  const x = axes.find((axis) => axis.name === "X")!;
  const y = axes.find((axis) => axis.name === "Y")!;
  const z = axes.find((axis) => axis.name === "Z")!;

  assert.deepEqual(x.screenDirection, [1, -0]);
  assert.deepEqual(y.screenDirection, [0, -1]);
  assert.deepEqual(z.screenDirection, [0, -0]);
  assert.equal(z.cameraDepth, 1);
});

test("world axis indicator follows camera rotation rather than screen coordinates", () => {
  const cameraRotation = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(0, 0, 1),
    Math.PI / 2,
  );
  const axes = projectWorldAxes(cameraRotation);
  const x = axes.find((axis) => axis.name === "X")!;
  const y = axes.find((axis) => axis.name === "Y")!;

  assert.ok(Math.abs(x.screenDirection[0]) < 1e-12);
  assert.ok(Math.abs(x.screenDirection[1] - 1) < 1e-12);
  assert.ok(Math.abs(y.screenDirection[0] - 1) < 1e-12);
  assert.ok(Math.abs(y.screenDirection[1]) < 1e-12);
});
