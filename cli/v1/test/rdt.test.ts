// The reference fixture is the cross-language arbiter (Python encoder, Go twin,
// C++ daemon): this reader must agree with its recorded digest and bytes.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import path from "node:path";
import { test } from "node:test";

import { decode, Rejection, summarize } from "../src/lib/rdt.js";
import { REPO_ROOT } from "./fakes.js";

const testdata = path.join(REPO_ROOT, "motion-server", "joint-trajectory", "v1", "testdata");
const fixture = readFileSync(path.join(testdata, "dense_joint_trajectory_reference.rdt"));
const expected = JSON.parse(readFileSync(path.join(testdata, "dense_joint_trajectory_reference.expected.json"), "utf8")) as {
  trajectory_digest: string;
  total_sample_count: number;
  segment_sample_counts: number[];
  blob_bytes: number;
  block_sha256: string[];
};

test("reference fixture: digest, block hashes and counts equal the recorded arbiter values", () => {
  assert.equal(fixture.byteLength, expected.blob_bytes);
  const { header, segments } = decode(fixture);
  assert.equal(header.trajectory_digest, expected.trajectory_digest);
  assert.deepEqual(segments.map((s) => s.samples.length), expected.segment_sample_counts);
  assert.deepEqual(header.segments.map((s) => s.block.sha256), expected.block_sha256.map((h) => `sha256:${h}`));
  const summary = summarize(fixture);
  assert.equal(summary.valid, true);
  assert.equal(summary.total_samples, expected.total_sample_count);
  assert.equal(summary.identity?.trajectory_digest, expected.trajectory_digest);
});

test("one flipped sample byte is refused per segment before any sample is parsed", () => {
  const corrupt = Buffer.from(fixture);
  const headerLen = corrupt.readUInt32BE(0);
  // First byte of segment 1's block (segment 0 holds 5 samples of 153 bytes).
  const offset = 4 + headerLen + 5 * 153;
  corrupt[offset] = corrupt[offset]! ^ 0xff;
  assert.throws(() => decode(corrupt), (error: unknown) => error instanceof Rejection && error.reason === "block_sha256_mismatch" && error.segmentIndex === 1);
  const summary = summarize(corrupt);
  assert.equal(summary.valid, false);
  assert.deepEqual(summary.rejection && [summary.rejection.reason, summary.rejection.segment_index], ["block_sha256_mismatch", 1]);
});

test("a header claiming more bytes than the blob is header_truncated", () => {
  assert.throws(() => decode(fixture.subarray(0, 40)), (error: unknown) => error instanceof Rejection && error.reason === "header_truncated");
});

test("a stored digest that is not the content digest is refused last, by name", () => {
  const headerLen = fixture.readUInt32BE(0);
  const header = JSON.parse(fixture.subarray(4, 4 + headerLen).toString("utf8")) as Record<string, unknown>;
  header.trajectory_digest = `sha256:${"0".repeat(64)}`;
  const json = Buffer.from(JSON.stringify(header), "utf8");
  const length = Buffer.alloc(4);
  length.writeUInt32BE(json.length);
  const forged = Buffer.concat([length, json, fixture.subarray(4 + headerLen)]);
  assert.throws(() => decode(forged), (error: unknown) => error instanceof Rejection && error.reason === "trajectory_digest_mismatch");
});
