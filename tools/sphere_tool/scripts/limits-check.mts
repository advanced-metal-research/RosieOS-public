/**
 * Forward kinematics must pose the robot the operator is looking at.
 *
 * `kinematics.clampToLimits` holds every joint inside `cell.joints[].limits_rad`
 * before solving. The limit editor deliberately keeps its edits OUT of that
 * field, so the export can diff against the URDF's own answer -- which means
 * anything that poses the cell has to be handed the edited bounds explicitly.
 * It was not, and widening a bound then driving past the committed stop left
 * the slider and the readout saying one angle while the viewport drew another.
 *
 *   node --experimental-strip-types scripts/limits-check.mts
 */

import { linkTransforms, type Cell } from "../src/kinematics.ts";
import { boundsOf, cellWithBounds, boundMoved } from "../src/limits.ts";

const deg = (d: number) => (d * Math.PI) / 180;
const fail = (why: string) => {
  console.error(`limits-check: FAIL ${why}`);
  process.exit(1);
};

// A two-link arm is enough: one joint with a stop, one fixed frame past it.
const cell: Cell = {
  frames: { base: "base", tool: "tool0" },
  links: [{ name: "base" }, { name: "arm" }, { name: "tool0" }],
  joints: [
    {
      name: "J1", type: "revolute", parent: "base", child: "arm",
      xyz_m: [0, 0, 0], rpy_rad: [0, 0, 0], axis_xyz: [0, 1, 0],
      limits_rad: { lower: deg(-80), upper: deg(80) },
    },
    {
      name: "fixed", type: "fixed", parent: "arm", child: "tool0",
      xyz_m: [1, 0, 0], rpy_rad: [0, 0, 0], axis_xyz: [0, 0, 1], limits_rad: null,
    },
  ],
} as unknown as Cell;

const x = (c: Cell, pose: Record<string, number>) =>
  linkTransforms(c, pose).get("tool0")![3];

const past = { J1: deg(100) };                  // past the declared 80 deg stop

// 1. the cell's own limits clamp, which is what they are for
const clamped = x(cell, past);
if (Math.abs(clamped - Math.cos(deg(80))) > 1e-9) {
  fail(`the declared limit did not clamp: tool0.x = ${clamped}`);
}

// 2. widened bounds must actually reach the pose the panel reports
const widened = boundsOf(cell);
widened.set("J1", { lower: deg(-80), upper: deg(120) });
const honoured = x(cellWithBounds(cell, widened), past);
if (Math.abs(honoured - Math.cos(deg(100))) > 1e-9) {
  fail(`edited bounds were ignored by FK: tool0.x = ${honoured}, want ${Math.cos(deg(100))}`);
}
if (Math.abs(honoured - clamped) < 1e-6) {
  fail("widening a bound changed nothing, so FK is still reading the cell's own limits");
}

// 3. the view must not disturb what the export diffs against
if (cell.joints[0].limits_rad!.upper !== deg(80)) {
  fail("cellWithBounds mutated the cell it was given");
}
if (!boundMoved(cell, "J1", widened)) fail("a widened bound was not reported as moved");
if (boundMoved(cell, "J1", boundsOf(cell))) fail("an untouched bound was reported as moved");

console.log(
  `limits-check: ok (clamped ${clamped.toFixed(4)}, honoured ${honoured.toFixed(4)}, ` +
  `${Math.abs(honoured - clamped).toFixed(4)} apart)`);
