import test from "node:test";
import assert from "node:assert/strict";
import type { Material, Object3D } from "three";
import { disposeObjectTree } from "./resource-disposal.ts";

test("object disposal releases shared geometry, texture, and material once", () => {
  const counts = { geometry: 0, texture: 0, material: 0 };
  const geometry = { dispose: () => { counts.geometry += 1; } };
  const texture = { isTexture: true, dispose: () => { counts.texture += 1; } };
  const material = {
    map: texture,
    dispose: () => { counts.material += 1; },
  } as unknown as Material;
  const nodes = [
    { geometry, material },
    { geometry, material },
  ];
  const root = {
    traverse(callback: (node: unknown) => void) {
      nodes.forEach(callback);
    },
  } as unknown as Object3D;

  disposeObjectTree(root);
  assert.deepEqual(counts, { geometry: 1, texture: 1, material: 1 });
});
