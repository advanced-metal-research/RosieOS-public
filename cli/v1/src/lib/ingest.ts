// The dense trajectory daemon's TCP ingest (motion-server/joint-trajectory/v1,
// framing mirrored from offline-programming/v1/internal/denseexec/ingest.go):
//
//   [u64 BE size][payload]    payload = [u32 BE json_len][JSON][blob]
//
// One request, one response, close. Only `status` and `validate` are sent
// from here: both are inert (validate is upload's dry run, nothing stored).
// `upload`/`preload` stage plans for play and belong to OLP's Load.
import net from "node:net";

/** The daemon's kMaxFrameBytes; a larger reply is refused before allocation. */
export const MAX_FRAME_BYTES = 64 * 1024 * 1024;

export type IngestKind = "status" | "validate";

export function encodeFrame(control: Record<string, unknown>, body: Uint8Array = new Uint8Array()): Buffer {
  const json = Buffer.from(JSON.stringify(control), "utf8");
  const payloadLen = 4 + json.length + body.byteLength;
  if (payloadLen > MAX_FRAME_BYTES) throw new Error(`frame of ${payloadLen} bytes exceeds ${MAX_FRAME_BYTES}`);
  const header = Buffer.alloc(12);
  header.writeBigUInt64BE(BigInt(payloadLen), 0);
  header.writeUInt32BE(json.length, 8);
  return Buffer.concat([header, json, body]);
}

/** Parses one reply frame; returns null until the whole frame has arrived. */
export function decodeFrame(buffer: Buffer): { control: unknown; body: Buffer } | null {
  if (buffer.length < 8) return null;
  const size = buffer.readBigUInt64BE(0);
  if (size < 4n || size > BigInt(MAX_FRAME_BYTES)) throw new Error(`reply frame ${size} bytes is outside 4..${MAX_FRAME_BYTES}`);
  const total = 8 + Number(size);
  if (buffer.length < total) return null;
  const jsonLen = buffer.readUInt32BE(8);
  if (4 + jsonLen > Number(size)) throw new Error(`reply json_len ${jsonLen} exceeds payload ${size}`);
  const control: unknown = JSON.parse(buffer.subarray(12, 12 + jsonLen).toString("utf8"));
  return { control, body: buffer.subarray(12 + jsonLen, total) };
}

export async function ingestExchange(
  addr: { host: string; port: number },
  kind: IngestKind,
  body: Uint8Array | undefined,
  timeoutMs: number,
): Promise<unknown> {
  const frame = encodeFrame({ kind }, body);
  return new Promise((resolve, reject) => {
    const socket = net.connect({ host: addr.host, port: addr.port });
    let received = Buffer.alloc(0);
    let settled = false;
    const finish = (error: Error | null, value?: unknown) => {
      if (settled) return;
      settled = true;
      socket.destroy();
      if (error) reject(error);
      else resolve(value);
    };
    socket.setTimeout(timeoutMs, () => finish(new Error(`dense daemon at ${addr.host}:${addr.port} did not answer ${kind} within ${timeoutMs} ms`)));
    socket.on("connect", () => socket.write(frame));
    socket.on("data", (chunk: Buffer) => {
      received = Buffer.concat([received, chunk]);
      try {
        const reply = decodeFrame(received);
        if (reply) finish(null, reply.control);
      } catch (error) {
        finish(error as Error);
      }
    });
    socket.on("error", (error) => finish(new Error(`dense daemon at ${addr.host}:${addr.port} is unreachable: ${error.message}`)));
    socket.on("end", () => finish(new Error(`dense daemon at ${addr.host}:${addr.port} closed before a whole reply`)));
  });
}
