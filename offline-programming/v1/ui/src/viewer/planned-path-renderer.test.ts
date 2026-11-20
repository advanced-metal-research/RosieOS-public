import assert from "node:assert/strict";
import test from "node:test";
import * as THREE from "three";
import { createPlannedPathGroup } from "./overlays.ts";
import type { MotionPathPoint } from "./geometry.ts";
import { WorkstationRenderer } from "./workstation-renderer.ts";
import { disposeObjectTree } from "./resource-disposal.ts";

const point = (x: number, motion: "weld" | "travel" = "travel"): MotionPathPoint => ({ positionM: [x, 0, 0], motion });

test("planned route renders travel and weld lines through the endpoints without linking segment gaps", () => {
  const group = createPlannedPathGroup([
    [point(0), point(1, "weld"), point(2)],
    [point(10), point(11)],
  ]);
  try {
    const lines = group.children as THREE.Line<THREE.BufferGeometry, THREE.LineBasicMaterial>[];
    assert.equal(lines.length, 3);
    assert.deepEqual(lines.map(line => line.userData.motion), ["travel", "weld", "travel"]);
    assert.deepEqual(lines.map(line => Array.from(line.geometry.getAttribute("position").array)), [
      [0, 0, 0, 1, 0, 0], [1, 0, 0, 2, 0, 0], [10, 0, 0, 11, 0, 0],
    ]);
    assert.notEqual(lines[0]!.material.color.getHex(), lines[1]!.material.color.getHex());
    assert.ok(group.children.every(child => child instanceof THREE.Line), "dense samples must not create waypoint-dot clutter");
  } finally { disposeObjectTree(group); }
});

test("planned visibility survives replacement, stays in world space and leaves authored overlays and camera alone", () => {
  const scene = new THREE.Scene();
  const workFrameLayer = new THREE.Group();
  const authored = new THREE.Group();
  const initial = new THREE.Group();
  scene.add(workFrameLayer, authored, initial);
  const fake = {
    scene, plannedPathLayer: initial, plannedPathPoints: [] as MotionPathPoint[], plannedPathVisible: true,
    elements: { stage: { dataset: {} as Record<string, string> } },
    assertActive() {},
    replaceLayer(current: THREE.Group, next: THREE.Group) {
      const parent = current.parent!;
      parent.remove(current); disposeObjectTree(current); current.clear(); parent.add(next); return next;
    },
  };
  const renderer = fake as unknown as WorkstationRenderer;
  try {
    WorkstationRenderer.prototype.setPlannedPath.call(renderer, [[point(4), point(6)]]);
    assert.equal(fake.plannedPathLayer.visible, true);
    assert.equal(fake.plannedPathLayer.parent, scene);
    const line = fake.plannedPathLayer.children[0] as THREE.Line;
    const before = line.getWorldPosition(new THREE.Vector3());
    workFrameLayer.position.set(50, 0, 0);
    workFrameLayer.rotation.z = Math.PI / 2;
    scene.updateMatrixWorld(true);
    assert.ok(line.getWorldPosition(new THREE.Vector3()).equals(before));
    let disposed = false;
    line.geometry.addEventListener("dispose", () => { disposed = true; });
    WorkstationRenderer.prototype.setPlannedPathVisible.call(renderer, false);
    assert.equal(fake.plannedPathLayer.visible, false);
    assert.equal(authored.visible, true);
    WorkstationRenderer.prototype.setPlannedPath.call(renderer, [[point(10), point(12)]]);
    assert.equal(disposed, true);
    assert.equal(fake.plannedPathLayer.visible, false, "a new plan must not undo the user's hidden choice");
    WorkstationRenderer.prototype.setPlannedPathVisible.call(renderer, true);
    assert.equal(fake.plannedPathLayer.visible, true);
    assert.equal(fake.elements.stage.dataset.plannedPathPointCount, "2");
    WorkstationRenderer.prototype.setPlannedPath.call(renderer, []);
    assert.equal(fake.plannedPathLayer.children.length, 0);
    assert.equal(fake.elements.stage.dataset.plannedPathPointCount, "0");
  } finally { disposeObjectTree(scene); }
});

test("explicit Frame includes the full planned route only when it is visible", () => {
  const camera = new THREE.PerspectiveCamera(46, 1, 0.01, 200);
  camera.position.set(0, -5, 3);
  const target = new THREE.Vector3();
  const fake = {
    focusTween: null,
    orbitCamera: camera,
    plannedPathVisible: true,
    plannedPathPoints: [point(-5), point(25)],
    controls: { target, update() {} },
    elements: { stage: { getBoundingClientRect: () => ({ width: 1000, height: 700 }) } },
  };
  WorkstationRenderer.prototype.framePath.call(fake as unknown as WorkstationRenderer, [point(0), point(2)]);
  assert.deepEqual(target.toArray(), [10, 0, 0]);
  fake.plannedPathVisible = false;
  WorkstationRenderer.prototype.framePath.call(fake as unknown as WorkstationRenderer, [point(0), point(2)]);
  assert.deepEqual(target.toArray(), [1, 0, 0]);
});
