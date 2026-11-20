import assert from "node:assert/strict";
import test from "node:test";

import type { RailsClearance } from "./client.ts";
import { conserves, narrowsToRun, offer, restrictScalars } from "./decisions.ts";

const clearance = (partial: Partial<RailsClearance>): RailsClearance => ({
  seam_length_m: 0.1,
  ...partial,
});

test("a seam blocked mid-way offers one split span per usable run", () => {
  const report = offer(
    clearance({ clear_runs: [[0, 0.4], [0.6, 1]], blocked_spans: [[0.4, 0.6]] }),
    { start_s: 0, end_s: 1 },
  );
  assert.equal(report.state, "partial");
  assert.equal(report.split.length, 2);
  assert.deepEqual(report.split[0], { start_s: 0, end_s: 0.4 });
  assert.deepEqual(report.split[1], { start_s: 0.6, end_s: 1 });
  // trim keeps only the longest run
  assert.deepEqual(report.trim, { start_s: 0, end_s: 0.4 });
  assert.ok(conserves(report));
});

test("runs shorter than the floor are dropped, not welded", () => {
  const report = offer(
    clearance({
      clear_runs: [[0, 0.02], [0.5, 1]],
      blocked_spans: [[0.02, 0.5]],
    }),
    { start_s: 0, end_s: 1 },
  );
  // 2 mm of a 100 mm seam is under the 5 mm floor
  assert.equal(report.usableRuns.length, 1);
  assert.equal(report.droppedRuns.length, 1);
  assert.equal(report.split.length, 1);
  assert.ok(conserves(report));
});

test("a fully clear seam has nothing to decide", () => {
  const report = offer(
    clearance({ clear_runs: [[0, 1]], blocked_spans: [] }),
    { start_s: 0, end_s: 1 },
  );
  assert.equal(report.state, "clear");
});

test("a run wrapping through zero on a closed seam keeps its length", () => {
  const report = offer(
    clearance({ clear_runs: [[0.8, 0.3]], blocked_spans: [[0.3, 0.8]] }),
    { start_s: 0, end_s: 1 },
  );
  assert.equal(report.state, "partial");
  assert.ok(Math.abs(report.usableLengthM - 0.05) < 1e-12);
  assert.ok(conserves(report));
});

test("split spans map through a weld span that does not start at zero", () => {
  const report = offer(
    clearance({ clear_runs: [[0, 0.5], [0.75, 1]], blocked_spans: [[0.5, 0.75]] }),
    { start_s: 0.2, end_s: 0.6 },
  );
  // fraction 0.5 of the [0.2, 0.6] span is s = 0.4
  assert.deepEqual(report.split[0], { start_s: 0.2, end_s: 0.4 });
  assert.deepEqual(report.split[1], { start_s: 0.5, end_s: 0.6 });
});

test("restrictScalars narrows every profile in the overrides tree", () => {
  const overrides = {
    orientation: {
      work_angle_deg: {
        kind: "piecewise_linear",
        knots: [[0, 0], [1, 10]],
      },
    },
    direction: "forward",
  };
  const narrowed = restrictScalars(overrides, 0.5, 1) as typeof overrides;
  const fn = narrowed.orientation.work_angle_deg as {
    kind: string;
    knots: number[][];
  };
  assert.equal(fn.kind, "piecewise_linear");
  assert.deepEqual(fn.knots[0], [0, 5]);
  assert.deepEqual(fn.knots[fn.knots.length - 1], [1, 10]);
  assert.equal(narrowed.direction, "forward");
});

test("one clear run with blocked metal at both ends still narrows to the run", () => {
  // The case that was falling through. Measured on `dog_box.step` seam_0017:
  // the search finds 11.9% weldable, the clearance check at those angles
  // reports one clear run between two blocked spans. Asking for two runs
  // before narrowing put the WHOLE seam in the list, fouling included.
  const report = offer(
    clearance({ clear_runs: [[0.35, 0.55]], blocked_spans: [[0, 0.35], [0.55, 1]] }),
    { start_s: 0, end_s: 1 },
  );
  assert.equal(report.state, "partial");
  assert.equal(report.split.length, 1);
  assert.equal(narrowsToRun(report), true);
  assert.deepEqual(report.split[0], { start_s: 0.35, end_s: 0.55 });
  assert.ok(conserves(report));
});

test("a seam with nothing blocked does not narrow", () => {
  // There is nothing to leave out, and splitting a weld with no problem is
  // how a single pass becomes two arc strikes for no reason.
  const report = offer(clearance({ clear_runs: [[0, 1]], blocked_spans: [] }), { start_s: 0, end_s: 1 });
  assert.equal(report.state, "clear");
  assert.equal(narrowsToRun(report), false);
});

test("a seam with no clear run anywhere does not narrow", () => {
  const report = offer(clearance({ clear_runs: [], blocked_spans: [[0, 1]] }), { start_s: 0, end_s: 1 });
  assert.equal(report.state, "blocked");
  assert.equal(narrowsToRun(report), false);
});

test("runs too short to strike an arc in do not make a seam narrowable", () => {
  // Dropped runs are not usable runs: narrowing to one would put a 1 mm
  // weld in the list.
  const report = offer(
    clearance({ seam_length_m: 0.1, clear_runs: [[0.5, 0.51]], blocked_spans: [[0, 0.5], [0.51, 1]] }),
    { start_s: 0, end_s: 1 },
    0.005,
  );
  assert.equal(report.usableRuns.length, 0);
  assert.equal(report.state, "blocked");
  assert.equal(narrowsToRun(report), false);
  assert.ok(conserves(report));
});
