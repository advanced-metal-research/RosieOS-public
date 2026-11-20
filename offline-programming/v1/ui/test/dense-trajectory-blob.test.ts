import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { fileURLToPath } from "node:url";

import {
  AXES,
  DenseBlobError,
  RECORD_BYTES,
  decodeDenseTrajectory,
  flattenForPlayback,
} from "../src/execution/denseTrajectoryBlob.ts";

/** The same arbiter the Go twin answers to. If this reader and the fixture
 *  disagree, this reader is wrong -- or the format changed and the fixture was
 *  regenerated, which is a decision and not an accident. */
function fixture(name: string): string {
  return fileURLToPath(new URL(`../../../../motion-server/joint-trajectory/v1/testdata/${name}`, import.meta.url));
}

async function reference(): Promise<{ blob: Uint8Array; expected: Record<string, unknown> }> {
  const blob = new Uint8Array(await readFile(fixture("dense_joint_trajectory_reference.rdt")));
  const expected = JSON.parse(
    await readFile(fixture("dense_joint_trajectory_reference.expected.json"), "utf8"),
  ) as Record<string, unknown>;
  return { blob, expected };
}

test("the reference blob decodes to the counts the fixture declares", async () => {
  const { blob, expected } = await reference();
  const trajectory = decodeDenseTrajectory(blob);

  assert.equal(trajectory.totalSampleCount, expected.total_sample_count);
  assert.deepEqual(
    trajectory.segments.map((segment) => segment.samples.length),
    expected.segment_sample_counts,
  );
  assert.equal(trajectory.trajectoryDigest, expected.trajectory_digest);
  assert.equal(blob.byteLength, expected.blob_bytes);
});

test("every sample carries nine axes, their rates, and the torch", async () => {
  const { blob } = await reference();
  const trajectory = decodeDenseTrajectory(blob);
  for (const segment of trajectory.segments) {
    for (const sample of segment.samples) {
      assert.equal(sample.qRad.length, AXES);
      assert.equal(sample.qdRadS.length, AXES);
      assert.equal(typeof sample.torchOn, "boolean");
      assert.ok(Number.isFinite(sample.tS), "a sample with no time is not a sample");
      for (const value of sample.qRad) assert.ok(Number.isFinite(value));
    }
  }
});

test("each segment keeps its own clock, starting at zero", async () => {
  const { blob } = await reference();
  // The mistake the encoder names outright: gluing the blocks onto one
  // timeline. A player walks segments in order; a boundary is a stop.
  for (const segment of decodeDenseTrajectory(blob).segments) {
    const first = segment.samples[0];
    if (!first) continue;
    assert.equal(first.tS, 0, `segment ${segment.index} does not start its own clock at zero`);
    let previous = -1;
    for (const sample of segment.samples) {
      assert.ok(sample.tS > previous, "a segment's samples are not in time order");
      previous = sample.tS;
    }
  }
});

test("playback time runs across the whole thing without touching the contract", async () => {
  const { blob, expected } = await reference();
  const trajectory = decodeDenseTrajectory(blob);
  const flat = flattenForPlayback(trajectory);

  assert.equal(flat.length, expected.total_sample_count);
  let previous = -1;
  for (const step of flat) {
    assert.ok(step.elapsedS >= previous, "playback time went backwards");
    previous = step.elapsedS;
  }
  // The samples themselves still carry the segment's own clock: the elapsed
  // time is a scrubber's, and nothing that commands motion reads it.
  assert.equal(trajectory.segments[0]!.samples[0]!.tS, 0);
});

test("a blob that disagrees with its own header is refused, not truncated", async () => {
  const { blob } = await reference();
  // Half a motion is not a shorter motion: a player handed one would run the
  // arm somewhere nobody planned.
  assert.throws(() => decodeDenseTrajectory(blob.subarray(0, blob.byteLength - RECORD_BYTES)),
    DenseBlobError);
  assert.throws(() => decodeDenseTrajectory(new Uint8Array([0, 0, 0])), DenseBlobError);
  const badHeader = new Uint8Array(8);
  new DataView(badHeader.buffer).setUint32(0, 0xffff, false);
  assert.throws(() => decodeDenseTrajectory(badHeader), DenseBlobError);
});

test("trailing bytes the header does not account for are refused", async () => {
  const { blob } = await reference();
  const longer = new Uint8Array(blob.byteLength + 4);
  longer.set(blob);
  assert.throws(() => decodeDenseTrajectory(longer), /disagree/);
});
