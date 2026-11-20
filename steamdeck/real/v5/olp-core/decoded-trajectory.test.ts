import assert from "node:assert/strict";
import test from "node:test";
import { decodeDenseTrajectory, RECORD_BYTES } from "../../../../offline-programming/v1/ui/src/execution/denseTrajectoryBlob.ts";
import { DecodedTrajectoryCache, MAX_DECODED_TRAJECTORY_BYTES, MAX_DECODED_TRAJECTORY_SAMPLES } from "./decoded-trajectory.ts";

function blob(samples = 2, marker = "a", padding = 0) {
  const header = Buffer.from(JSON.stringify({ schema: "dense", trajectory_digest: "sha256:same-claimed-digest",
    marker, padding: " ".repeat(padding), segments: [{ sample_count: samples }] }));
  const bytes = Buffer.alloc(4 + header.length + samples * RECORD_BYTES);
  bytes.writeUInt32BE(header.length); header.copy(bytes, 4);
  return bytes.toString("base64");
}

test("direct bytes equal browser atob for every one/two-byte value, lengths, padding and ASCII whitespace", () => {
  const empty = decodeDenseTrajectory(Buffer.from(blob(0), "base64"));
  let received = new Uint8Array(), sequence = 0;
  const cache = new DecodedTrajectoryCache(bytes => { received = bytes; return empty; });
  const check = (text: string) => {
    const expected = Uint8Array.from(atob(text), char => char.charCodeAt(0));
    cache.read(String(sequence++), text);assert.deepEqual(received, expected);
  };
  for (let value = 0; value < 256; ++value) check(Buffer.from([value]).toString("base64"));
  for (let value = 0; value < 65536; ++value) check(Buffer.from([value >> 8, value & 255]).toString("base64"));
  for (let length = 0; length <= 260; ++length) {
    const text = Buffer.from(Array.from({length}, (_, index) => (index * 127 + length) & 255)).toString("base64");
    check(text);check(text.replace(/=+$/, ""));
    check("\t " + text.split("").join("\r\n\f") + " \n");
  }
});

test("malformed base64 never reaches the dense decoder or leaves a prior cache entry", () => {
  const empty = decodeDenseTrajectory(Buffer.from(blob(0), "base64"));
  let decoded = 0, sequence = 0;
  const cache = new DecodedTrajectoryCache(() => { ++decoded; return empty; });
  const check = (text: string) => {
    let valid = true;try { atob(text); } catch { valid = false; }
    const before = decoded;
    if (valid) {cache.read(String(sequence++), text);assert.equal(decoded, before + 1);}
    else {assert.throws(() => cache.read(String(sequence++), text), /Invalid base64/);assert.equal(decoded, before);}
  };
  // Enumerate each remainder and padding count, including excess/interior '='.
  for (let length = 0; length < 13; ++length) for (let padding = 0; padding < 6; ++padding) {
    check("A".repeat(length) + "=".repeat(padding));
    check("A".repeat(length) + "=".repeat(padding) + "A");
  }
  for (let code = 0; code < 256; ++code) check("A" + String.fromCharCode(code) + "==");
  for (const text of ["AA-_", "AA\u0100=", "AA\ufeff=", "AA\u2028=", "AA\ud800=", "AA\u00a0=", "AA==\v"]) check(text);
  cache.read("kept", "AA==");const before = decoded;
  assert.throws(() => cache.read("kept", "AA=bad"), /Invalid base64/);
  cache.read("kept", "AA==");assert.equal(decoded, before + 1);
});

test("one exact byte/identity match reuses decoding; other requests replace the sole entry", () => {
  let decodes = 0;
  const cache = new DecodedTrajectoryCache(bytes => { ++decodes; return decodeDenseTrajectory(bytes); });
  const a = blob(), b = blob(2, "b");
  const first = cache.read("program-a/rev1/robot", a);
  assert.equal(cache.read("program-a/rev1/robot", a), first);
  assert.equal(decodes, 1);
  assert.notEqual(cache.read("program-a/rev1/robot", b), first); // identical digest, different actual bytes
  assert.notEqual(cache.read("program-a/rev1/robot", a), first); // only one resident entry
  assert.equal(decodes, 3);
  cache.read("program-b/rev1/robot", a);
  cache.read("program-b/rev2/robot", a);
  cache.read("program-b/rev2/changed-robot", a);
  assert.equal(decodes, 6);
  assert.throws(() => cache.read("program-b/rev2/changed-robot", "bad!"));
  cache.read("program-b/rev2/changed-robot", a);
  assert.equal(decodes, 7); // malformed requests evict, never revive a prior value
  assert.throws(() => cache.read("program-b/rev2/changed-robot", Buffer.from([0, 0, 0, 99]).toString("base64")), /header/);
  cache.read("program-b/rev2/changed-robot", a);
  assert.equal(decodes, 9);
});

test("byte and sample bounds independently prevent retention without truncating or refusing valid plans", () => {
  const cache = new DecodedTrajectoryCache();
  const small = blob();
  const first = cache.read("a", small);
  const tooMany = blob(MAX_DECODED_TRAJECTORY_SAMPLES + 1);
  const many = cache.read("a", tooMany);
  assert.equal(many.totalSampleCount, MAX_DECODED_TRAJECTORY_SAMPLES + 1);
  assert.notEqual(cache.read("a", tooMany), many);
  assert.notEqual(cache.read("a", small), first);
  const largeHeader = blob(1, "x", MAX_DECODED_TRAJECTORY_BYTES);
  const large = cache.read("a", largeHeader);
  assert.equal(large.totalSampleCount, 1);
  assert.notEqual(cache.read("a", largeHeader), large);
  const boundary = blob(MAX_DECODED_TRAJECTORY_SAMPLES);
  const kept = cache.read("a", boundary);
  assert.equal(cache.read("a", boundary), kept);
  assert.equal(kept.totalSampleCount, MAX_DECODED_TRAJECTORY_SAMPLES);
});

test("owned native buffers match base64, reuse only the same source/identity, and retain the same bounds", () => {
  const binary = (text: string): ArrayBuffer => Uint8Array.from(Buffer.from(text, "base64")).buffer;
  let decodes = 0;
  const cache = new DecodedTrajectoryCache(bytes => { ++decodes; return decodeDenseTrajectory(bytes); });
  const encoded = blob(), source = binary(encoded);
  const expected = cache.read("program-a/calibration-a", encoded);
  const native = cache.read("program-a/calibration-a", source);
  assert.deepEqual(native, expected);
  const before = decodes;
  assert.equal(cache.read("program-a/calibration-a", source), native);
  assert.equal(decodes, before);
  // A new native source, even with the same length/claimed digest, cannot
  // alias the prior decode. Qt owns and compares the actual source bytes.
  const changed = new Uint8Array(source.slice(0)); changed[changed.length - 1] = 1;
  assert.notDeepEqual(cache.read("program-a/calibration-a", changed.buffer), native);
  assert.notEqual(cache.read("program-b/calibration-a", source), native);
  const second = cache.read("program-b/calibration-a", source);
  assert.notEqual(cache.read("program-b/calibration-b", source), second);
  assert.throws(() => cache.read("program-b/calibration-b", new Uint8Array([0, 0, 0, 99]).buffer), /header/);
  assert.deepEqual(cache.read("program-a/calibration-a", source), expected);
  for (const oversized of [binary(blob(MAX_DECODED_TRAJECTORY_SAMPLES + 1)), binary(blob(1, "x", MAX_DECODED_TRAJECTORY_BYTES))]) {
    const read = cache.read("large", oversized);
    assert.notEqual(cache.read("large", oversized), read);
  }
  const boundary = binary(blob(MAX_DECODED_TRAJECTORY_SAMPLES));
  const kept = cache.read("boundary", boundary);
  assert.equal(cache.read("boundary", boundary), kept);
});
