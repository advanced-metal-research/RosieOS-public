import assert from "node:assert/strict";
import test from "node:test";
import { Vector3 } from "three";

import { closestAxisParameter, ringAngle, wrapAngle } from "./part-gizmo.ts";

// The triad reads a drag off the pointer ray alone: an arrow as the closest
// point on its axis line, a ring as the angle in its plane. These pin the
// two readings in absolute units so a drag of 0.30 m along X is 0.30 m, not
// whatever the projection happens to give.

const origin = new Vector3(1, 2, 0.5);
const X = new Vector3(1, 0, 0);
const Z = new Vector3(0, 0, 1);

function rayThrough(from: Vector3, to: Vector3) {
  return { origin: from.clone(), direction: to.clone().sub(from).normalize() };
}

test("an arrow drag reads the axis parameter of the point under the pointer", () => {
  const eye = new Vector3(1, -3, 2);
  // Aim the ray at a point 0.30 m along X from the anchor.
  const target = origin.clone().addScaledVector(X, 0.3);
  const t = closestAxisParameter(rayThrough(eye, target), origin, X);
  assert.ok(t !== null);
  assert.ok(Math.abs(t - 0.3) < 1e-9, `parameter ${t} should be 0.30`);
});

test("an arrow drag ignores pointer motion off the axis", () => {
  const eye = new Vector3(1, -3, 2);
  const target = origin.clone().addScaledVector(X, 0.3).add(new Vector3(0, 0, 0.4));
  // The ray misses the axis; the closest point is still where it passes X.
  const t = closestAxisParameter(rayThrough(eye, target), origin, X);
  assert.ok(t !== null);
  assert.ok(Math.abs(t - 0.3) < 0.06, `parameter ${t} should stay near 0.30`);
});

test("a ray along the axis has no arrow reading", () => {
  const eye = origin.clone().addScaledVector(X, -5);
  assert.equal(closestAxisParameter(rayThrough(eye, origin), origin, X), null);
});

test("a ring drag reads the angle of the pointer's hit in the ring plane", () => {
  const eye = new Vector3(0, 0, 5).add(origin);
  // A hit at 90° around Z lands on +Y when the basis's e1 is +X.
  const target = origin.clone().add(new Vector3(0, 0.2, 0));
  const a = ringAngle(rayThrough(eye, target), origin, Z);
  const b = ringAngle(rayThrough(eye, origin.clone().add(new Vector3(0.2, 0, 0))), origin, Z);
  assert.ok(a !== null && b !== null);
  const turn = wrapAngle(a - b);
  assert.ok(Math.abs(Math.abs(turn) - Math.PI / 2) < 1e-9, `turn ${turn} should be a quarter`);
});

test("a ring seen edge-on has no reading", () => {
  const eye = origin.clone().add(new Vector3(0, -5, 0));
  assert.equal(ringAngle(rayThrough(eye, origin), origin, Z), null);
});

test("wrapAngle keeps a step within a half turn", () => {
  assert.ok(Math.abs(wrapAngle(Math.PI * 1.5) + Math.PI / 2) < 1e-12);
  assert.ok(Math.abs(wrapAngle(-Math.PI * 1.5) - Math.PI / 2) < 1e-12);
  assert.equal(wrapAngle(0.25), 0.25);
});
