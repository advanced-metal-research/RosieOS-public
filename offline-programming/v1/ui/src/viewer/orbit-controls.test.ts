import assert from "node:assert/strict";
import test from "node:test";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { Window } from "happy-dom";
import { WorkstationOrbitControls } from "./orbit-controls.ts";

function harness(t: import("node:test").TestContext, legacy = false) {
  const window = new Window();
  const pane = window.document.createElement("div"); window.document.body.append(pane);
  pane.getBoundingClientRect = () => new window.DOMRect(0, 0, 1000, 700);
  Object.defineProperty(pane, "clientWidth", { value: 1000 });
  Object.defineProperty(pane, "clientHeight", { value: 700 });
  pane.setPointerCapture = () => {}; pane.releasePointerCapture = () => {};
  const camera = new THREE.PerspectiveCamera(45, 1000 / 700, 0.01, 100);
  camera.up.set(0, 0, 1); camera.position.set(0, -5, 2);
  const controls = legacy
    ? new OrbitControls(camera, pane as unknown as HTMLElement)
    : new WorkstationOrbitControls(camera, pane as unknown as HTMLElement);
  controls.enableDamping = true; controls.dampingFactor = 0.08;
  controls.update();
  let starts = 0, ends = 0, pointerUps = 0, clicks = 0;
  controls.addEventListener("start", () => { starts += 1; });
  controls.addEventListener("end", () => { ends += 1; });
  pane.addEventListener("pointerup", () => { pointerUps += 1; });
  pane.addEventListener("click", () => { clicks += 1; });
  t.after(async () => { controls.dispose(); await window.happyDOM.close(); });
  function pointer(type: string, x: number, buttons = 1, button = 0, modifiers: { ctrlKey?: boolean } = {}) {
    const event = new window.PointerEvent(type, { clientX: x, clientY: 350, pointerId: 1, pointerType: "mouse", button, buttons, bubbles: true, ...modifiers });
    Object.defineProperties(event, { pageX: { value: x }, pageY: { value: 350 } });
    pane.dispatchEvent(event);
  }
  const settle = () => { for (let i = 0; i < 300; i++) controls.update(); };
  function drag(button = 0, modifiers = {}) {
    const start = camera.quaternion.clone(), target = controls.target.clone();
    pointer("pointerdown", 500, 1, button, modifiers);
    pointer("pointermove", 600, 1, button, modifiers);
    pointer("pointerup", 600, 0, button, modifiers); settle();
    return { angle: start.angleTo(camera.quaternion), pan: target.distanceTo(controls.target) };
  }
  return { window, pane, camera, controls, pointer, settle, drag, counts: () => ({ starts, ends, pointerUps, clicks }) };
}

for (const loss of ["lostpointercapture", "blur", "buttons0", "pointercancel"] as const) {
  test(`${loss} ends the gesture without a click and the next drag starts normally`, t => {
    const h = harness(t);
    h.pointer("pointerdown", 500); h.pointer("pointermove", 550); h.settle();
    assert.equal((h.controls as WorkstationOrbitControls).interacting, true);
    if (loss === "blur") h.window.dispatchEvent(new h.window.Event("blur"));
    else h.pointer(loss === "buttons0" ? "pointermove" : loss, 550, 0);
    h.settle();
    assert.equal((h.controls as WorkstationOrbitControls).interacting, false);
    assert.deepEqual(h.counts(), { starts: 1, ends: 1, pointerUps: 0, clicks: 0 });
    const before = h.camera.quaternion.clone();
    h.pointer("pointermove", 750, 0); h.settle();
    assert.ok(before.angleTo(h.camera.quaternion) < 1e-7, "hover must not continue the lost drag");
    const moved = h.drag();
    assert.ok(moved.angle > 0.89 && moved.angle < 0.91, "100px drag retains ordinary Orbit rotation sensitivity");
    assert.equal(h.counts().starts, 2); assert.equal(h.counts().ends, 2);
  });
}

test("legacy Orbit reproduces lost-capture hover movement and ignored next press", t => {
  const h = harness(t, true);
  h.pointer("pointerdown", 500); h.pointer("pointermove", 550); h.settle();
  h.pointer("lostpointercapture", 550, 0);
  const before = h.camera.quaternion.clone(); h.pointer("pointermove", 650, 0); h.settle();
  assert.ok(before.angleTo(h.camera.quaternion) > 0.89);
  h.pointer("pointerdown", 500); assert.equal(h.counts().starts, 1);
});

test("ordinary Orbit modifier/right pan, damping and upright orientation remain unchanged", t => {
  const h = harness(t);
  assert.equal(h.controls.enableDamping, true); assert.equal(h.controls.dampingFactor, 0.08);
  for (const [button, modifiers] of [[2, {}], [0, { ctrlKey: true }]] as const) {
    const moved = h.drag(button, modifiers); assert.ok(moved.pan > 0.6); assert.ok(moved.angle < 1e-7);
  }
  assert.deepEqual(h.camera.up.toArray(), [0, 0, 1]);
  assert.equal(h.controls.minPolarAngle, 0); assert.equal(h.controls.maxPolarAngle, Math.PI);
});

test("a fresh press recovers a missed release and disposal removes recovery listeners", t => {
  const h = harness(t);
  h.pointer("pointerdown", 500); h.pointer("pointermove", 550); h.settle();
  h.pointer("pointerdown", 500);
  assert.deepEqual(h.counts(), { starts: 2, ends: 1, pointerUps: 0, clicks: 0 });
  h.controls.dispose();
  assert.equal((h.controls as WorkstationOrbitControls).interacting, false);
  const counts = h.counts(), before = h.camera.position.clone();
  h.pointer("pointerdown", 500); h.pointer("pointermove", 600);
  h.window.dispatchEvent(new h.window.Event("blur")); h.settle();
  assert.deepEqual(h.counts(), counts);
  assert.ok(before.distanceTo(h.camera.position) < 1e-7);
  assert.equal((h.controls as WorkstationOrbitControls).interacting, false);
});

for (const captured of [false, true]) {
  test(`lost-capture recovery tolerates an inactive browser pointer (capture=${captured})`, t => {
    const h = harness(t);
    h.pointer("pointerdown", 500); h.pointer("pointermove", 550); h.settle();
    h.pane.hasPointerCapture = () => captured;
    const release = () => { throw new h.window.DOMException("Pointer no longer active", "NotFoundError"); };
    h.pane.releasePointerCapture = release;
    h.pointer("lostpointercapture", 550, 0);
    assert.equal(h.counts().ends, 1, "native cancellation must complete after capture has already vanished");
    assert.equal(h.pane.releasePointerCapture, release, "release override must be restored synchronously");
    assert.equal((h.controls as WorkstationOrbitControls).interacting, false);
    h.pane.releasePointerCapture = () => {};
    assert.ok(h.drag().angle > 0.89);
  });
}

test("viewport text drag cannot hand camera input to the browser drag/drop loop", t => {
  const h = harness(t);
  const label = h.window.document.createElement("span"); label.textContent = "ORBIT"; h.pane.append(label);
  h.pointer("pointerdown", 500); h.pointer("pointermove", 502); h.controls.update();
  const drag = new h.window.DragEvent("dragstart", { bubbles: true, cancelable: true });
  assert.equal(label.dispatchEvent(drag), false, "native dragstart must be cancelled before the browser takes over");
  assert.equal(drag.defaultPrevented, true);
  assert.equal((h.controls as WorkstationOrbitControls).interacting, true, "camera drag continues");
  h.pointer("pointermove", 600); h.pointer("pointerup", 600, 0); h.settle();
  assert.equal(h.counts().ends, 1);
  const input = h.window.document.createElement("input"); h.pane.append(input);
  const editing = new h.window.DragEvent("dragstart", { bubbles: true, cancelable: true });
  assert.equal(input.dispatchEvent(editing), true, "text editing in a viewport card keeps its native drag behavior");
  h.controls.dispose();
  const released = new h.window.DragEvent("dragstart", { bubbles: true, cancelable: true });
  assert.equal(label.dispatchEvent(released), true, "dispose removes drag prevention");
});
