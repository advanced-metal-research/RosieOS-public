import test from "node:test";
import assert from "node:assert/strict";
import {
  boundsOfPoints,
  buildMotionRuns,
  polylineLengthM,
  type MotionPathPoint,
} from "./geometry.ts";

test("motion runs keep the shared transition vertex", () => {
  const points: MotionPathPoint[] = [
    { positionM: [0, 0, 0], motion: "travel" },
    { positionM: [1, 0, 0], motion: "weld" },
    { positionM: [1, 1, 0], motion: "weld" },
    { positionM: [2, 1, 0], motion: "travel" },
  ];
  const runs = buildMotionRuns(points);
  assert.deepEqual(runs, [
    { motion: "travel", pointsM: [[0, 0, 0], [1, 0, 0]] },
    { motion: "weld", pointsM: [[1, 0, 0], [1, 1, 0], [2, 1, 0]] },
  ]);
});

test("geometry length and bounds use meter-space points", () => {
  const points = [[0, 0, 0], [3, 4, 0], [3, 4, 12]] as const;
  assert.equal(polylineLengthM(points), 17);
  assert.deepEqual(boundsOfPoints(points), {
    minimum: [0, 0, 0],
    maximum: [3, 4, 12],
    center: [1.5, 2, 6],
    size: [3, 4, 12],
  });
});

test("geometry rejects non-finite coordinates", () => {
  assert.throws(
    () => buildMotionRuns([
      { positionM: [0, 0, 0], motion: "travel" },
      { positionM: [Number.NaN, 0, 0], motion: "travel" },
    ]),
    /non-finite/,
  );
});
