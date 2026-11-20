import { decodeDenseTrajectory, type DenseTrajectory } from "../../../../offline-programming/v1/ui/src/execution/denseTrajectoryBlob.ts";

// One small/medium plan only. Larger plans still use the complete decoder,
// but their expanded samples are not retained between UI operations.
export const MAX_DECODED_TRAJECTORY_BYTES = 2 * 1024 * 1024;
export const MAX_DECODED_TRAJECTORY_SAMPLES = 8192;
// ArrayBuffers come only from OlpCore's owned, read-only native-byte bridge.
// Consumers never mutate or return them; string callers retain atob semantics.
export type TrajectorySource = string | ArrayBuffer;

const alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
const codes = new Uint8Array(256); codes.fill(255);
for (let index = 0; index < alphabet.length; ++index) codes[alphabet.charCodeAt(index)] = index;

// Browser atob semantics, directly into bytes. Qt's compatibility atob first
// built one JS string per byte, joined them, then copied every character here.
// Avoid that UI-thread allocation/iteration without changing the dense reader.
function bytesOf(base64: string): Uint8Array {
  const text = base64.replace(/[\t\n\f\r ]/g, "");
  const padding = text.endsWith("==") ? 2 : text.endsWith("=") ? 1 : 0;
  const end = text.length - padding;
  if (end % 4 === 1 || (padding && (text.length % 4 !== 0 || end < 2))) throw new Error("Invalid base64 length");
  const bytes = new Uint8Array(Math.floor(end * 6 / 8));
  let value = 0, bits = 0, index = 0;
  for (let at = 0; at < end; ++at) {
    const code = text.charCodeAt(at), part = code < 256 ? codes[code]! : 255;
    if (part === 255) throw new Error("Invalid base64 character");
    value = ((value << 6) | part) & 0xffffff;
    bits += 6;
    if (bits >= 8) { bits -= 8; bytes[index++] = (value >> bits) & 255; }
  }
  return bytes;
}

/** Private-to-OLP read cache: consumers must not mutate the decoded value.
 * Live robot/calibration checks remain outside it and run on every use.
 * Exact bytes, not only the claimed digest, identify the cached trajectory. */
export class DecodedTrajectoryCache {
  private entry: { identity: string; source: TrajectorySource; trajectory: DenseTrajectory } | null = null;
  private readonly decode: (bytes: Uint8Array) => DenseTrajectory;
  constructor(decode: (bytes: Uint8Array) => DenseTrajectory = decodeDenseTrajectory) { this.decode = decode; }

  read(identity: string, source: TrajectorySource): DenseTrajectory {
    if (this.entry?.identity === identity && this.entry.source === source) return this.entry.trajectory;
    // A changed or malformed request cannot leave an older plan resident.
    this.entry = null;
    const bytes = typeof source === "string" ? bytesOf(source) : new Uint8Array(source);
    const trajectory = this.decode(bytes);
    if (identity.length <= 4096 && (typeof source !== "string" || source.length <= 4 * Math.ceil(MAX_DECODED_TRAJECTORY_BYTES / 3)) &&
        bytes.byteLength <= MAX_DECODED_TRAJECTORY_BYTES && trajectory.totalSampleCount <= MAX_DECODED_TRAJECTORY_SAMPLES) {
      this.entry = { identity, source, trajectory };
    }
    return trajectory;
  }
}
