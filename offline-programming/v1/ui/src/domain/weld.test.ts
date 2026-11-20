import assert from "node:assert/strict";
import test from "node:test";

import type { Rpy, Vec3 } from "./project.ts";
import {
  torchOrientationRpy,
  type ResolvedTorchSettings,
} from "./weld.ts";

const DEG = Math.PI / 180;
const ZERO: ResolvedTorchSettings = {
  workAngleDeg: 0,
  travelAngleDeg: 0,
  spinAngleDeg: 0,
};

test("zero torch angles on a +X seam point tool +X straight down", () => {
  const rpy = requiredOrientation([1, 0, 0], ZERO);
  closeVec(rotateLocalAxis(rpy, [1, 0, 0]), [0, 0, -1]);
  // The secondary-axis convention keeps local +Y along seam travel.
  closeVec(rotateLocalAxis(rpy, [0, 1, 0]), [1, 0, 0]);
});

test("pure work angle tilts across the seam with no along-travel component", () => {
  const rpy = requiredOrientation([1, 0, 0], {
    ...ZERO,
    workAngleDeg: 30,
  });
  const torch = rotateLocalAxis(rpy, [1, 0, 0]);
  closeVec(torch, [0, 0.5, -Math.sqrt(3) / 2]);
  close(torch[0], 0);
});

test("positive pure travel angle tips the torch direction backward", () => {
  const rpy = requiredOrientation([1, 0, 0], {
    ...ZERO,
    travelAngleDeg: 30,
  });
  closeVec(
    rotateLocalAxis(rpy, [1, 0, 0]),
    [-0.5, 0, -Math.sqrt(3) / 2],
  );
});

test("spin changes the tilted torch azimuth about world up", () => {
  const rpy = requiredOrientation([1, 0, 0], {
    ...ZERO,
    workAngleDeg: 30,
    spinAngleDeg: 90,
  });
  closeVec(
    rotateLocalAxis(rpy, [1, 0, 0]),
    [-0.5, 0, -Math.sqrt(3) / 2],
  );
});

test("pure spin remains observable in the secondary axis for a vertical torch", () => {
  const rpy = requiredOrientation([1, 0, 0], {
    ...ZERO,
    spinAngleDeg: 90,
  });
  closeVec(rotateLocalAxis(rpy, [1, 0, 0]), [0, 0, -1]);
  closeVec(rotateLocalAxis(rpy, [0, 1, 0]), [0, 1, 0]);
});

test("reversing seam direction reverses work and travel tilt sides", () => {
  const forward = requiredOrientation([1, 0, 0], {
    ...ZERO,
    workAngleDeg: 30,
    travelAngleDeg: 20,
  });
  const reversed = requiredOrientation([-1, 0, 0], {
    ...ZERO,
    workAngleDeg: 30,
    travelAngleDeg: 20,
  });
  const forwardTorch = rotateLocalAxis(forward, [1, 0, 0]);
  const reversedTorch = rotateLocalAxis(reversed, [1, 0, 0]);
  close(reversedTorch[0], -forwardTorch[0]);
  close(reversedTorch[1], -forwardTorch[1]);
  close(reversedTorch[2], forwardTorch[2]);
  const reversedSecondary = rotateLocalAxis(reversed, [0, 1, 0]);
  assert.ok(
    reversedSecondary[0] < -0.9,
    "the projected local +Y secondary axis must follow reversed seam travel",
  );
});

test("vertical and degenerate seam tangents fail closed", () => {
  assert.equal(torchOrientationRpy([0, 0, 1], ZERO), null);
  assert.equal(torchOrientationRpy([0, 0, -1], ZERO), null);
  assert.equal(torchOrientationRpy([0, 0, 0], ZERO), null);
  assert.equal(torchOrientationRpy([Number.NaN, 0, 0], ZERO), null);
});

function requiredOrientation(
  travel: Vec3,
  torch: ResolvedTorchSettings,
): Rpy {
  const orientation = torchOrientationRpy(travel, torch);
  if (!orientation) assert.fail("expected a valid torch orientation");
  return orientation;
}

function rotateLocalAxis(rpy: Rpy, local: Vec3): Vec3 {
  const [roll, pitch, yaw] = rpy;
  const cr = Math.cos(roll);
  const sr = Math.sin(roll);
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  return [
    (cy * cp) * local[0] +
      (cy * sp * sr - sy * cr) * local[1] +
      (cy * sp * cr + sy * sr) * local[2],
    (sy * cp) * local[0] +
      (sy * sp * sr + cy * cr) * local[1] +
      (sy * sp * cr - cy * sr) * local[2],
    (-sp) * local[0] + (cp * sr) * local[1] + (cp * cr) * local[2],
  ];
}

function closeVec(actual: Vec3, expected: Vec3, tolerance = 1e-9): void {
  actual.forEach((value, index) => close(value, expected[index]!, tolerance));
}

function close(actual: number, expected: number, tolerance = 1e-9): void {
  assert.ok(
    Math.abs(actual - expected) <= tolerance,
    `expected ${actual} to be within ${tolerance} of ${expected} (${30 * DEG} rad reference)`,
  );
}
