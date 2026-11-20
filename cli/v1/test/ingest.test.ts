import assert from "node:assert/strict";
import { once } from "node:events";
import net from "node:net";
import { test } from "node:test";

import { decodeFrame, encodeFrame, ingestExchange } from "../src/lib/ingest.js";

test("request frame is [u64 BE size][u32 BE json_len][JSON][blob], as the daemon reads it", () => {
  const blob = Uint8Array.from([1, 2, 3]);
  const frame = encodeFrame({ kind: "validate" }, blob);
  const json = Buffer.from(JSON.stringify({ kind: "validate" }));
  assert.equal(frame.readBigUInt64BE(0), BigInt(4 + json.length + 3));
  assert.equal(frame.readUInt32BE(8), json.length);
  assert.equal(frame.subarray(12, 12 + json.length).toString(), json.toString());
  assert.deepEqual([...frame.subarray(12 + json.length)], [1, 2, 3]);
});

test("a reply frame claiming more than 64 MiB is refused before allocation", () => {
  const header = Buffer.alloc(8);
  header.writeBigUInt64BE(65n * 1024n * 1024n);
  assert.throws(() => decodeFrame(header), /outside 4\.\./);
});

test("status exchange against a fake daemon: one request, one reply, close", async () => {
  const seen: unknown[] = [];
  const server = net.createServer((socket) => {
    let buffer = Buffer.alloc(0);
    socket.on("data", (chunk: Buffer) => {
      buffer = Buffer.concat([buffer, chunk]);
      const request = decodeFrame(buffer);
      if (!request) return;
      seen.push(request.control);
      socket.end(encodeFrame({ ok: true, executor: { state: "idle" } }));
    });
  });
  server.listen(0, "127.0.0.1");
  await once(server, "listening");
  const { port } = server.address() as net.AddressInfo;
  try {
    const reply = await ingestExchange({ host: "127.0.0.1", port }, "status", undefined, 2000);
    assert.deepEqual(seen, [{ kind: "status" }]);
    assert.deepEqual(reply, { ok: true, executor: { state: "idle" } });
  } finally {
    server.close();
  }
});
