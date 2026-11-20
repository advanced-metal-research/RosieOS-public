import assert from "node:assert/strict";
import test from "node:test";
import * as THREE from "three";
import { workpieceTopology } from "../../test/support/workpiece-fixtures.ts";
import { appendCadQueryTopology } from "../adapters/cadquery.ts";
import { createProject } from "../domain/project.ts";
import { PartGizmo } from "./part-gizmo.ts";
import { disposeObjectTree } from "./resource-disposal.ts";
import { syncWorkpieceMesh } from "./workstation.ts";
import { WorkstationRenderer, type WorkstationEvent } from "./workstation-renderer.ts";

/** Real scene, handles and pointer picking, without allocating WebGL. */
function harness() {
  const pane = Object.assign(new EventTarget(), {
    style: { cursor: "" },
    getBoundingClientRect: () => ({ left: 0, top: 0, width: 800, height: 600 }),
  });
  const camera = new THREE.PerspectiveCamera(46, 800 / 600, 0.01, 200);
  camera.position.set(3, -4, 3);
  camera.lookAt(0, 0, 0);
  camera.updateMatrixWorld(true);
  const scene = new THREE.Scene();
  const pivot = new THREE.Object3D();
  const layer = new THREE.Group();
  scene.add(layer, pivot);
  const events: WorkstationEvent[] = [];
  const state = {
    scene, orbitCamera: camera, workpieceLayer: layer, waypointLayer: new THREE.Group(),
    gizmoPivot: pivot, gizmo: null as PartGizmo | null, gizmoDrag: null,
    gizmoEnabled: false, gizmoHasAnchor: false, workpieceIdentity: "", disposed: false,
    controls: { enabled: true },
    elements: { stage: { dataset: {} }, orbitPane: pane as unknown as HTMLElement },
    onEvent: (event: WorkstationEvent) => events.push(event),
  };
  const renderer = Object.assign(Object.create(WorkstationRenderer.prototype), state) as WorkstationRenderer;
  const internals = renderer as unknown as typeof state;
  return {
    renderer, internals, events,
    pointer(type: string, world: THREE.Vector3) {
      const point = world.clone().project(camera);
      pane.dispatchEvent(Object.assign(new Event(type, { cancelable: true }), {
        button: 0, pointerId: 1, clientX: (point.x + 1) * 400, clientY: (1 - point.y) * 300,
      }));
    },
    close() { internals.gizmo?.dispose(); disposeObjectTree(scene); },
  };
}

test("an imported edge-only part has usable Move part arrows before its surface loads", () => {
  const h = harness();
  try {
    const project = appendCadQueryTopology(createProject({ robotModel: "rosie_1420_v1" }), workpieceTopology());
    syncWorkpieceMesh(h.renderer, project, { workpieceMesh: null });
    h.renderer.setWorkpieceGizmo(true);
    const gizmo = h.internals.gizmo!;
    assert.equal(gizmo.helper.visible, true);
    assert.equal(gizmo.object, h.internals.gizmoPivot);
    assert.equal(h.internals.workpieceLayer.children.length, 0, "surface is still unavailable");

    gizmo.update();
    gizmo.helper.updateMatrixWorld(true);
    const start = gizmo.helper.position.clone().add(new THREE.Vector3(gizmo.helper.scale.x * 0.5, 0, 0));
    h.pointer("pointerdown", start);
    assert.equal(gizmo.dragging, true, "the visible arrow must be pickable");
    h.pointer("pointermove", start.clone().add(new THREE.Vector3(0.2, 0, 0)));
    h.pointer("pointerup", start);
    const placed = h.events.find(event => event.type === "workpiece-placed");
    assert.ok(placed?.type === "workpiece-placed");
    assert.ok(Math.abs(placed.anchorM[0] - 0.2) < 1e-8, "drag commits the new placement without a surface");
    assert.equal(h.internals.controls.enabled, true);

    h.renderer.setWorkpiece(null, { originM: [1, 2, 0], rotationRpyRad: [0, 0, 0], anchorM: [1.1, 2, 0] });
    assert.deepEqual(h.internals.gizmoPivot.position.toArray(), [1.1, 2, 0], "numeric edits update the meshless anchor");
    h.renderer.setWorkpieceGizmo(false);
    assert.equal(gizmo.helper.visible, false);
    h.renderer.setWorkpieceGizmo(true);
    assert.equal(gizmo.helper.visible, true);
    h.renderer.setWorkpiece(null, null);
    assert.equal(gizmo.helper.visible, false, "clearing the part removes its handles");
    assert.equal(gizmo.object, null);
  } finally { h.close(); }
});
