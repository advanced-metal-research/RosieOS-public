import { test } from "node:test";
import assert from "node:assert/strict";
import { executionPathSamples, executionTcpErrors } from "./executedPath.ts";
import type { DenseCapabilities, DenseTelemetryWindow } from "./denseClient.ts";

function fixture() {
  const capabilities = { rtCore: { configurationSHA256: "config", axes: [
    { id: "J2", index: 0, position_unit: "rad", counts_per_unit: 100, sign: -1 },
    { id: "J1", index: 1, position_unit: "rad", counts_per_unit: 100, sign: 1 },
  ] } } as unknown as DenseCapabilities;
  const window = { daemon_incarnation: "daemon", configuration_sha256: "config", stride: 1, cycle_period_ns: 1e6, axis_count: 2,
    records: [1, 2, 3, 5].map(seq => ({ seq, t_ns: seq * 1e6, trajectory_id: 3, mode: 2, state: 3,
      home_epoch: 2, configuration_epoch: 1, submitted_mask: 3,
      ax: [0, 1].map(() => ({ p: 10, tp: 100 + seq, coordinate_counts: 110, home_offset: -100, coordinate_ok: true })),
    })) } as DenseTelemetryWindow;
  return { capabilities, window };
}

test("TCP capture uses absolute homed continuous coordinates, named joints and preceding submitted commands", () => {
  const { capabilities, window } = fixture();
  const trace = executionPathSamples(window, capabilities, "robot", ["J1", "J2"]);
  assert.deepEqual(trace.actual.map(r => r.length), [3, 1]);
  assert.deepEqual(trace.actual[0]![0]!.jointsRad, [0.1, -0.1]);
  assert.deepEqual(trace.commanded[0]![1]!.jointsRad, [0.01, -0.01]);
  assert.deepEqual(trace.comparable, [[false, true, true], [false]]);
  window.records[0]!.submitted_mask = 0;
  assert.equal(executionPathSamples(window, capabilities, "robot", ["J1", "J2"]).comparable[0]![1], false);
});

test("commanded capture keeps the continuous target without nearest-turn unwrapping", () => {
  const { capabilities, window } = fixture();
  window.records[1]!.ax[1]!.coordinate_counts = 800;
  const trace = executionPathSamples(window, capabilities, "robot", ["J1", "J2"]);
  assert.equal(trace.commanded[0]![1]!.jointsRad[0], 0.01);
});

test("invalid feedback splits paths; identity changes and reordered time are rejected", () => {
  const { capabilities, window } = fixture();
  window.records[1]!.ax[0]!.coordinate_ok = false;
  const trace = executionPathSamples(window, capabilities, "robot", ["J1", "J2"]);
  assert.equal(trace.omitted, 1);
  assert.deepEqual(trace.actual.map(r => r.length), [1, 1, 1]);
  window.records[2]!.home_epoch = 3;
  assert.throws(() => executionPathSamples(window, capabilities, "robot", ["J1", "J2"]), /Home or configuration/);
  window.records[2]!.home_epoch = 2;
  window.records[2]!.t_ns = 0;
  assert.throws(() => executionPathSamples(window, capabilities, "robot", ["J1", "J2"]), /strictly increasing/);
});

test("tracking metrics retain lag and exclude gaps rather than aligning paths to hide error", () => {
  const { capabilities, window } = fixture();
  const trace = executionPathSamples(window, capabilities, "robot", ["J1", "J2"]);
  const positions = (x: number) => trace.actual.map(run => run.map(() => ({ positionM: [x, 0, 0] as const, motion: "travel" as const })));
  const errors = executionTcpErrors(trace, positions(0), positions(0.012));
  assert.equal(errors.count, 2); assert.equal(errors.peakMM, 12); assert.equal(errors.rmsMM, 12);
});
