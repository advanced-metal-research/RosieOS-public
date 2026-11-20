import assert from "node:assert/strict";
import test from "node:test";

import { DenseSimulation } from "../src/execution/denseSimulation.ts";
import type { DenseSimulationSnapshot } from "../src/execution/denseSimulation.ts";
import type { SimulationSample } from "../src/simulation/contracts.ts";

/** A clock and a scheduler a test drives by hand. Nothing here waits. */
function harness() {
  let nowMs = 0;
  const pending: (() => void)[] = [];
  const shown: (SimulationSample | null)[] = [];
  const snapshots: DenseSimulationSnapshot[] = [];
  const simulation = new DenseSimulation({
    now: () => nowMs,
    schedule: (step) => pending.push(step),
    cancel: () => pending.splice(0, pending.length),
    show: (sample) => shown.push(sample),
    onChange: (snapshot) => snapshots.push(snapshot),
  });
  return {
    simulation, shown, snapshots,
    advance(seconds: number) {
      nowMs += seconds * 1000;
      const due = pending.splice(0, pending.length);
      for (const step of due) step();
    },
  };
}

function samples(times: number[]): SimulationSample[] {
  return times.map((timeS, index) => ({
    modelId: "rosie_1400_v3",
    jointOrder: ["J1"],
    jointsRad: [index * 0.1],
    source: "preview" as const,
    timeS,
    motion: "travel" as const,
  }));
}

test("loading shows the first sample and waits", () => {
  const { simulation, shown, snapshots } = harness();
  simulation.load(samples([0, 0.01, 0.02]), [false, true, false]);
  assert.equal(simulation.phase, "paused");
  assert.equal(shown.at(-1)!.timeS, 0);
  assert.equal(snapshots.at(-1)!.durationS, 0.02);
});

test("the clock is the trajectory's, so a slow frame drops samples", () => {
  // Not one sample per frame: at 100 Hz that would play a weld back at
  // whatever rate the browser felt like.
  const { simulation, shown, advance } = harness();
  simulation.load(samples([0, 0.01, 0.02, 0.03, 0.04]), [false, false, false, false, false]);
  simulation.play();
  shown.length = 0;
  advance(0.03);
  assert.equal(shown.at(-1)!.timeS, 0.03, "a 30 ms frame should land on the 30 ms sample");
});

test("a pose between two samples is a pose that never occurs", () => {
  // At or before the cursor, never interpolated: the samples are the
  // planner's own and the daemon plays them one at a time.
  const { simulation, shown, advance } = harness();
  simulation.load(samples([0, 0.1, 0.2]), [false, false, false]);
  simulation.play();
  shown.length = 0;
  advance(0.15);
  assert.equal(shown.at(-1)!.timeS, 0.1);
});

test("playing a finished trajectory starts it again", () => {
  const { simulation, advance, snapshots } = harness();
  simulation.load(samples([0, 0.01]), [false, false]);
  simulation.play();
  advance(1);
  assert.equal(simulation.phase, "ended");
  simulation.play();
  assert.equal(simulation.phase, "playing");
  assert.equal(snapshots.at(-1)!.cursorS, 0);
});

test("pause keeps the cursor and stop rewinds it", () => {
  const { simulation, advance, snapshots } = harness();
  simulation.load(samples([0, 0.1, 0.2]), [false, false, false]);
  simulation.play();
  advance(0.1);
  simulation.pause();
  assert.equal(simulation.phase, "paused");
  assert.ok(snapshots.at(-1)!.cursorS > 0);
  simulation.stop();
  assert.equal(snapshots.at(-1)!.cursorS, 0);
});

test("the torch flag follows the cursor", () => {
  const { simulation, advance, snapshots } = harness();
  simulation.load(samples([0, 0.1, 0.2]), [false, true, false]);
  simulation.play();
  advance(0.1);
  assert.equal(snapshots.at(-1)!.torchOn, true, "the arc should be lit on the welding sample");
  advance(0.1);
  assert.equal(snapshots.at(-1)!.torchOn, false);
});

test("seeking shows the sample it landed on without playing", () => {
  const { simulation, shown, snapshots } = harness();
  simulation.load(samples([0, 0.1, 0.2]), [false, false, false]);
  simulation.seek(0.2);
  assert.equal(shown.at(-1)!.timeS, 0.2);
  assert.equal(simulation.phase, "paused");
  simulation.seek(99);
  assert.equal(snapshots.at(-1)!.cursorS, 0.2, "seeking past the end clamps");
});

test("a playback rate scales the trajectory clock, not the samples", () => {
  // 4x: 10 ms of wall time lands on the 40 ms sample. The samples and their
  // own times are untouched; only how fast the cursor walks them changes.
  const { simulation, shown, snapshots, advance } = harness();
  simulation.load(samples([0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08]), Array(9).fill(false));
  simulation.setRate(4);
  assert.equal(snapshots.at(-1)!.rate, 4);
  simulation.play();
  shown.length = 0;
  advance(0.01);
  assert.equal(shown.at(-1)!.timeS, 0.04, "10 ms at 4x should land on the 40 ms sample");
  assert.equal(snapshots.at(-1)!.cursorS, 0.04);
});

test("changing the rate mid-play continues from the cursor", () => {
  // 20 ms at 1x, then 16x: the next 1 ms of wall time is 16 ms of trajectory
  // from 20 ms, so 36 ms -- not 16x of the whole elapsed time (336 ms).
  const { simulation, snapshots, advance } = harness();
  const times = Array.from({ length: 50 }, (_, index) => index * 0.01);
  simulation.load(samples(times), times.map(() => false));
  simulation.play();
  advance(0.02);
  assert.equal(snapshots.at(-1)!.cursorS, 0.02);
  simulation.setRate(16);
  assert.equal(snapshots.at(-1)!.cursorS, 0.02, "changing the rate does not move the cursor");
  advance(0.001);
  assert.ok(Math.abs(snapshots.at(-1)!.cursorS - 0.036) < 1e-9, `got ${snapshots.at(-1)!.cursorS}`);
});


test("move steps replay samples in both directions and stop exactly at boundaries", () => {
  const { simulation, shown, snapshots, advance } = harness();
  const path = samples([0, .5, 1, 1, 1.5, 2]);
  path.forEach((sample, i) => sample.nodeId = i < 3 ? "p1" : "p2");
  simulation.load(path, []);
  const boundaries = [0, 1, 2];
  assert.equal(simulation.canStep("backward", boundaries), false);
  simulation.step("forward", boundaries);
  advance(.5);
  assert.equal(shown.at(-1)?.timeS, .5);
  advance(2); // A slow frame must not overshoot the requested move.
  assert.equal(snapshots.at(-1)?.cursorS, 1);
  assert.equal(simulation.phase, "paused");
  assert.equal(shown.at(-1)?.nodeId, "p1");
  simulation.step("forward", boundaries);
  advance(2);
  assert.equal(snapshots.at(-1)?.cursorS, 2);
  assert.equal(simulation.canStep("forward", boundaries), false);
  simulation.step("backward", boundaries);
  advance(.5);
  assert.equal(shown.at(-1)?.timeS, 1.5);
  simulation.setRate(2);
  advance(.5);
  assert.equal(snapshots.at(-1)?.cursorS, 1);
  assert.equal(simulation.phase, "paused");
  simulation.step("backward", boundaries);
  advance(2);
  assert.equal(snapshots.at(-1)?.cursorS, 0);
  assert.equal(simulation.canStep("backward", boundaries), false);
  simulation.play();
  advance(.25);
  assert.equal(snapshots.at(-1)?.cursorS, .5, "Play resumes forward after reverse stepping");
  simulation.stop();
  advance(5);
  assert.equal(snapshots.at(-1)?.cursorS, 0);
});
