import assert from "node:assert/strict";
import test from "node:test";

import type { AuthoredSeam, Vec3 } from "./project.ts";
import {
  migrateLegacySeamTrims,
  pointOnPolylineNormalizedS,
  polylineLengthM,
  samplePolylineNormalizedWindow,
  seamTrimLimits,
  setSeamTrimOffsetM,
  trimPolylineByOffsetsM,
} from "./seam.ts";
import { createProject } from "./project.ts";

test("normalized polyline samples land on exact millimetre positions", () => {
  const points: Vec3[] = [[0, 0, 0], [0.3, 0, 0]];
  assert.deepEqual(pointOnPolylineNormalizedS(points, 0), [0, 0, 0]);
  assert.deepEqual(pointOnPolylineNormalizedS(points, 1), [0.3, 0, 0]);
  const mid = pointOnPolylineNormalizedS(points, 0.452);
  assert.ok(mid);
  assert.ok(Math.abs(mid[0] - 0.1356) < 1e-12);
  const window = samplePolylineNormalizedWindow(points, 0.452, 0.771, 3);
  assert.equal(window.length, 3);
  assert.ok(Math.abs(window[0]![0]! - 0.1356) < 1e-12);
  assert.ok(Math.abs(window[2]![0]! - 0.2313) < 1e-12);
});

test("physical seam trims interpolate exact millimetre endpoints", () => {
  const points: Vec3[] = [
    [0, 0, 0],
    [0.3, 0, 0],
    [0.3, 0.4, 0],
  ];
  assert.ok(Math.abs(polylineLengthM(points) - 0.7) < 1e-12);
  const trimmed = trimPolylineByOffsetsM(points, 0.15, 0.1);
  const expected: Vec3[] = [
    [0.15, 0, 0],
    [0.3, 0, 0],
    [0.3, 0.3, 0],
  ];
  assert.equal(trimmed.length, expected.length);
  trimmed.forEach((point, pointIndex) => {
    point.forEach((value, axis) => {
      assert.ok(
        Math.abs(value - expected[pointIndex]![axis]!) < 1e-12,
        `${value} != ${expected[pointIndex]![axis]} at ${pointIndex}:${axis}`,
      );
    });
  });
});

test("exact trim entry is clamped without allowing the path to collapse", () => {
  const seam = testSeam();
  setSeamTrimOffsetM(seam, "start", 0.7);
  setSeamTrimOffsetM(seam, "end", 0.7);
  const limits = seamTrimLimits(seam);
  assert.ok(limits.startOffsetM >= 0);
  assert.ok(limits.endOffsetM >= 0);
  assert.ok(limits.remainingLengthM >= 0.0001 - 1e-12);
});

test("legacy normalized trim windows migrate to explicit metre offsets", () => {
  const project = createProject();
  const seam = testSeam();
  seam.trim = { start: 0.1, end: 0.8 } as unknown as AuthoredSeam["trim"];
  project.seams = [seam];
  assert.equal(migrateLegacySeamTrims(project), true);
  assert.ok(Math.abs(project.seams[0]!.trim.startOffsetM - 0.1) < 1e-12);
  assert.ok(Math.abs(project.seams[0]!.trim.endOffsetM - 0.2) < 1e-12);
});

function testSeam(): AuthoredSeam {
  return {
    id: "seam-1",
    label: "PATH 1",
    enabled: true,
    sourceEdgeId: "edge-1",
    partId: "part-1",
    geometryType: "LINE",
    trim: { startOffsetM: 0, endOffsetM: 0 },
    reversed: false,
    geometry: {
      kind: "polyline",
      pointsM: [[0, 0, 0], [1, 0, 0]],
    },
    workFrameId: "world",
    processParameterId: "process-default",
  };
}
