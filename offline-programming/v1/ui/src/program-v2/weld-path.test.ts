import assert from "node:assert/strict";
import test from "node:test";

import type { Curve, WeldNode } from "../contracts/robot-v4-program-v2.ts";
import { manualPoseWeldNode } from "../waypoints/node.ts";
import { TORCH_DOWN, type V3 } from "../waypoints/model.ts";
import { curveSpan, evaluate, weldPathPoints } from "./weld-path.ts";

// A 40 mm circle as the standard seven-point rational quadratic, the form a
// CAD arc arrives in (hitch_2x seam_0008), and a 100 mm square.
const R = 0.04;
const S3 = Math.sqrt(3);
const CIRCLE: Curve = {
  closed: true,
  segments: [{
    degree: 2,
    knots_u: [0, 0, 0, 1, 1, 2, 2, 3, 3, 3],
    control_points_xyz_m: [[1, 0], [1, S3], [-0.5, S3 / 2], [-2, 0], [-0.5, -S3 / 2], [1, -S3], [1, 0]]
      .map(([x, y]) => [R * x!, R * y!, 0] as V3),
    weights: [1, 0.5, 1, 0.5, 1, 0.5, 1],
  }],
};

const line = (a: V3, b: V3) => ({ degree: 1, knots_u: [0, 0, 1, 1], control_points_xyz_m: [a, b] });
const SQUARE: Curve = {
  closed: true,
  segments: [
    line([0, 0, 0], [0.1, 0, 0]),
    line([0.1, 0, 0], [0.1, 0.1, 0]),
    line([0.1, 0.1, 0], [0, 0.1, 0]),
    line([0, 0.1, 0], [0, 0, 0]),
  ],
};

const near = (a: V3, b: V3, tol = 1e-9) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) < tol;

test("a rational segment evaluates onto its circle", () => {
  for (let k = 0; k <= 30; k += 1) {
    const p = evaluate(CIRCLE.segments[0]!, (3 * k) / 30);
    assert.ok(Math.abs(Math.hypot(...p) - R) < 1e-12, `u=${(3 * k) / 30}: r=${Math.hypot(...p)}`);
  }
});

test("a span is cut at its arc length, and a wrapping one goes through s = 0", () => {
  // A quarter of the square's 400 mm starts 50 mm along its first side.
  const quarter = curveSpan(SQUARE, 0.125, 0.375);
  assert.ok(near(quarter[0]!, [0.05, 0, 0]));
  assert.ok(near(quarter.at(-1)!, [0.1, 0.05, 0]));
  assert.ok(quarter.some((p) => near(p, [0.1, 0, 0])), "the corner is kept");
  // 0.875 -> 0.125 wraps: from the middle of the last side through the origin.
  const wrap = curveSpan(SQUARE, 0.875, 0.125);
  assert.ok(near(wrap[0]!, [0, 0.05, 0]));
  assert.ok(near(wrap.at(-1)!, [0.05, 0, 0]));
  assert.ok(wrap.some((p) => near(p, [0, 0, 0])));
});

test("a hand-placed weld draws its own path; a classic one draws nothing here", () => {
  const points = [[0, 0, 0], [0.03, 0, 0]].map((positionM, i) => ({
    id: `p${i}`, positionM: positionM as V3, quaternion: [...TORCH_DOWN] as [number, number, number, number],
    continuity: "c0" as const, standoffMm: 17,
  }));
  const node = manualPoseWeldNode("L", points, { travelSpeedMmS: 8, weave: { shape: "none" } }, { tool: "line" });
  const path = weldPathPoints(node)!;
  assert.ok(near(path[0]!, [0, 0, 0]) && near(path.at(-1)!, [0.03, 0, 0]));
  const classic = { ...node, geometry: { kind: "posed_polyline", frame_id: "world", points: [] } } as unknown as WeldNode;
  assert.equal(weldPathPoints(classic), null);
});
