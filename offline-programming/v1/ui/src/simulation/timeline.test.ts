import test from "node:test";
import assert from "node:assert/strict";
import type { SimulationSample } from "./contracts.ts";
import {
  PlaybackClock,
  interpolateSample,
  sampleTimeline,
  validateTimeline,
} from "./timeline.ts";
import {
  TimedPlayback,
  type AnimationScheduler,
} from "./timed-playback.ts";

const ORDER = ["J1", "J2"] as const;

function sample(
  timeS: number,
  jointsRad: readonly [number, number],
  motion: "travel" | "weld" = "travel",
): SimulationSample<"test_robot"> {
  return {
    modelId: "test_robot",
    jointOrder: ORDER,
    jointsRad,
    source: "preview",
    timeS,
    tcpPose: [timeS, 0, 0, 0, 0, 0],
    motion,
  };
}

test("timeline interpolation blends pose and retains segment motion", () => {
  const blended = interpolateSample(
    sample(0, [0, 2], "weld"),
    sample(2, [2, 4], "travel"),
    0.25,
  );
  assert.deepEqual(blended.jointsRad, [0.5, 2.5]);
  assert.equal(blended.tcpPose?.[0], 0.5);
  assert.equal(blended.motion, "weld");
  assert.equal(blended.source, "playback");
});

test("timeline lookup clamps ends and interpolates between timestamps", () => {
  const samples = [sample(0, [0, 0]), sample(2, [2, 4], "weld")];
  const middle = sampleTimeline(samples, 1);
  assert.equal(middle.done, false);
  assert.equal(middle.index, 0);
  assert.deepEqual(middle.sample?.jointsRad, [1, 2]);
  assert.equal(sampleTimeline(samples, 8).done, true);
});

test("timeline validation rejects identity drift and non-monotonic time", () => {
  const invalid: SimulationSample[] = [
    sample(2, [0, 0]),
    { ...sample(1, [1, 1]), modelId: "other_robot" },
  ];
  const result = validateTimeline(invalid);
  assert.equal(result.ok, false);
  assert.match(result.errors.join("\n"), /not monotonic/);
  assert.match(result.errors.join("\n"), /modelId/);
});

test("playback clock supports rate, loop, seek, and terminal state", () => {
  const clock = new PlaybackClock<"test_robot">();
  clock.load([sample(0, [0, 0]), sample(2, [2, 2])]);
  clock.setPlaybackRate(2);
  clock.setLoop(true);
  clock.play();
  assert.equal(clock.advance(0.75).cursorS, 1.5);
  assert.equal(clock.advance(0.5).cursorS, 0.5);
  clock.setLoop(false);
  clock.seek(1.8);
  clock.play();
  const ended = clock.advance(0.2);
  assert.equal(ended.cursorS, 2);
  assert.equal(ended.status, "ended");
});

test("timed playback schedules frames and cancels them on dispose", () => {
  let nowMs = 1000;
  let nextHandle = 1;
  const callbacks = new Map<number, (now: number) => void>();
  const scheduler: AnimationScheduler = {
    now: () => nowMs,
    request: (callback) => {
      const handle = nextHandle;
      nextHandle += 1;
      callbacks.set(handle, callback);
      return handle;
    },
    cancel: (handle) => {
      callbacks.delete(handle);
    },
  };
  const snapshots: number[] = [];
  const playback = new TimedPlayback<"test_robot">(
    (snapshot) => snapshots.push(snapshot.cursorS),
    scheduler,
  );
  playback.load([sample(0, [0, 0]), sample(2, [2, 2])]);
  playback.play();
  const [handle, callback] = callbacks.entries().next().value as [number, (now: number) => void];
  callbacks.delete(handle);
  nowMs = 1500;
  callback(nowMs);
  assert.equal(snapshots.at(-1), 0.25); // driver clamps one frame delta to 250 ms
  assert.equal(callbacks.size, 1);
  playback.dispose();
  assert.equal(callbacks.size, 0);
  assert.throws(() => playback.play(), /disposed/);
});
