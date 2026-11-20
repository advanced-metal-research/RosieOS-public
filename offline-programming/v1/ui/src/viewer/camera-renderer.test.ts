import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import * as THREE from "three";
import ts from "typescript";
import { WorkstationRenderer, PART_FOCUS_OPACITY } from "./workstation-renderer.ts";

/** Run the real lifecycle methods with cameras and scene objects, without
 * constructing a WebGL renderer or loading robot assets. */
function harness() {
  const camera = new THREE.PerspectiveCamera(46, 1, 0.01, 200);
  camera.position.set(4, -5, 3);
  camera.up.set(0, 0, 1);
  const target = new THREE.Vector3(1, 1, 0);
  let updates = 0;
  const controls = new Proxy({ enabled: true, interacting: false, target, update: () => { updates++; camera.lookAt(target); } }, {
    set(object, key, value) {
      assert.ok(!["minPolarAngle", "maxPolarAngle", "minAzimuthAngle", "maxAzimuthAngle"].includes(String(key)),
        "part focus must not introduce angular limits");
      return Reflect.set(object, key, value);
    },
  });
  const scene = new THREE.Scene();
  const workFrameLayer = new THREE.Group();
  const workpieceLayer = new THREE.Group();
  const part = new THREE.Mesh(new THREE.BoxGeometry(1, 2, 0.5), new THREE.MeshBasicMaterial());
  part.position.set(2, 3, 1);
  workpieceLayer.add(part);
  workFrameLayer.add(workpieceLayer);
  scene.add(workFrameLayer);
  const surroundingMaterial = new THREE.MeshBasicMaterial({ opacity: 0.8 });
  scene.add(new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), surroundingMaterial));
  const fake = {
    orbitCamera: camera, controls, scene, workFrameLayer, workpieceLayer,
    gizmo: null, waypointGizmo: null, partFocus: null as unknown,
    focusTween: { active: true } as unknown,
    assertActive() {},
    pathLayer: new THREE.Group(),
    replaceLayer: (_previous: THREE.Group, next: THREE.Group) => { scene.add(next); return next; },
    descriptor: { worldUp: [0, 0, 1], orbitHome: { positionM: [6, -7, 5], targetM: [1, 2, 3] } },
    elements: { stage: { dataset: {} as Record<string, string>, getBoundingClientRect: () => ({ width: 1000, height: 700 }) } },
    framePart: () => WorkstationRenderer.prototype.framePart.call(fake as unknown as WorkstationRenderer),
    resetOrbitCamera: () => WorkstationRenderer.prototype.resetOrbitCamera.call(fake as unknown as WorkstationRenderer),
  };
  return { fake, camera, target, surroundingMaterial, part,
    updates: () => updates,
    asRenderer: fake as unknown as WorkstationRenderer,
    close: () => scene.traverse(node => {
      if (!(node instanceof THREE.Mesh)) return;
      node.geometry.dispose();
      for (const material of Array.isArray(node.material) ? node.material : [node.material]) material.dispose();
    }),
  };
}

for (const blockedBy of ["pointer interaction", "disabled controls"] as const) {
  test(`programmatic focus cannot compete with ${blockedBy} and resumes afterwards`, () => {
    const h = harness();
    try {
      const position = h.camera.position.clone();
      const target = h.target.clone();
      h.fake.focusTween = null;
      if (blockedBy === "pointer interaction") h.fake.controls.interacting = true;
      else h.fake.controls.enabled = false;
      for (const duration of [350, 0]) {
        WorkstationRenderer.prototype.focusOn.call(h.asRenderer, [[10, 20, 30]], duration);
        assert.equal(h.fake.focusTween, null, "background updates must not restart the camera animation");
        assert.ok(h.camera.position.equals(position));
        assert.ok(h.target.equals(target));
        assert.equal(h.updates(), 0);
      }
      h.fake.controls.interacting = false;
      h.fake.controls.enabled = true;
      WorkstationRenderer.prototype.focusOn.call(h.asRenderer, [[10, 20, 30]], 350);
      const tween = h.fake.focusTween as { toTarget: THREE.Vector3; fromPosition: THREE.Vector3 };
      assert.deepEqual(tween.toTarget.toArray(), [10, 20, 30]);
      assert.ok(tween.fromPosition.equals(position));
      assert.ok(h.camera.position.equals(position), "restored animated focus begins from the current camera");
    } finally { h.close(); }
  });
}

test("Home cancels pending camera movement and restores the model's upright camera and target", () => {
  const h = harness();
  try {
    h.fake.resetOrbitCamera();
    assert.equal(h.fake.focusTween, null);
    assert.deepEqual(h.camera.up.toArray(), [0, 0, 1]);
    assert.deepEqual(h.camera.position.toArray(), [6, -7, 5]);
    assert.deepEqual(h.target.toArray(), [1, 2, 3]);
    assert.equal(h.updates(), 1);
  } finally { h.close(); }
});

test("explicitly framing a part or path cancels a previous focus animation", () => {
  const h = harness();
  try {
    const up = h.camera.up.clone();
    h.fake.framePart();
    assert.equal(h.fake.focusTween, null);
    assert.deepEqual(h.target.toArray(), [2, 3, 1]);
    assert.ok(h.camera.up.equals(up));
    h.fake.focusTween = { active: true };
    WorkstationRenderer.prototype.framePath.call(h.asRenderer, [
      { positionM: [1, 2, 3], motion: "weld" },
      { positionM: [3, 4, 5], motion: "weld" },
    ]);
    assert.equal(h.fake.focusTween, null);
    assert.deepEqual(h.target.toArray(), [2, 3, 4]);
    assert.ok(h.camera.up.equals(up));
    assert.equal(h.updates(), 2);
  } finally { h.close(); }
});

test("part focus only dims surroundings and preserves the camera on entry and exit", () => {
  const h = harness();
  try {
    const position = h.camera.position.clone();
    const target = h.target.clone();
    const up = h.camera.up.clone();
    WorkstationRenderer.prototype.setPartFocus.call(h.asRenderer, true);
    assert.equal(h.surroundingMaterial.opacity, PART_FOCUS_OPACITY);
    assert.equal(h.part.material.opacity, 1, "the focused part must not be dimmed");
    assert.ok(h.camera.position.equals(position), "entering a weld tool must not move the camera");
    assert.ok(h.target.equals(target));
    assert.ok(h.camera.up.equals(up));
    h.camera.position.set(-8, 5, -3);
    h.target.set(-1, -2, -3);
    WorkstationRenderer.prototype.setPartFocus.call(h.asRenderer, false);
    assert.equal(h.fake.partFocus, null);
    assert.deepEqual(h.camera.position.toArray(), [-8, 5, -3], "leaving a weld tool preserves the user's new view");
    assert.deepEqual(h.target.toArray(), [-1, -2, -3]);
    assert.ok(h.camera.up.equals(up));
    assert.equal(h.surroundingMaterial.opacity, 0.8);
    assert.equal(h.surroundingMaterial.transparent, false);
    assert.equal(h.surroundingMaterial.depthWrite, true);
    assert.equal(h.updates(), 0);
  } finally { h.close(); }
});

test("updating the drawn path never frames or resets the camera", () => {
  const h = harness();
  try {
    const position = h.camera.position.clone();
    const target = h.target.clone();
    h.fake.resetOrbitCamera = () => { throw new Error("path refresh must not reset the camera"); };
    Object.assign(h.fake, { framePath: () => { throw new Error("path refresh must not frame the camera"); } });
    WorkstationRenderer.prototype.setPath.call(h.asRenderer, [{ positionM: [1, 2, 3], motion: "weld" }]);
    WorkstationRenderer.prototype.setPath.call(h.asRenderer, [
      { positionM: [50, 60, 70], motion: "weld" },
      { positionM: [90, 100, 110], motion: "weld" },
    ]);
    WorkstationRenderer.prototype.setPath.call(h.asRenderer, []);
    assert.ok(h.camera.position.equals(position));
    assert.ok(h.target.equals(target));
    assert.equal(h.updates(), 0);
  } finally { h.close(); }
});

test("model loading and spreadsheet selection contain no automatic camera framing", async () => {
  const rendererSource = await readFile(new URL("./workstation-renderer.ts", import.meta.url), "utf8");
  const ast = ts.createSourceFile("renderer.ts", rendererSource, ts.ScriptTarget.ES2022, true);
  const renderer = ast.statements.find(node => ts.isClassDeclaration(node) && node.name?.text === "WorkstationRenderer") as ts.ClassDeclaration;
  const load = renderer.members.find(node => ts.isMethodDeclaration(node) && node.name.getText(ast) === "loadModel")!;
  assert.doesNotMatch(load.getText(ast), /this\.(resetOrbitCamera|framePath|framePart|focusOn)\(/);
  const path = renderer.members.find(node => ts.isMethodDeclaration(node) && node.name.getText(ast) === "setPath")!;
  assert.doesNotMatch(path.getText(ast), /frameKey|framedPathKey|this\.(resetOrbitCamera|framePath|framePart|focusOn)\(/);
  const main = await readFile(new URL("../main.ts", import.meta.url), "utf8");
  assert.doesNotMatch(main, /viewer\.focusOn\(|followSelection/);
});
