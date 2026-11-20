// TypeScript twin of robot.v4.dense-joint-trajectory.v1
// (motion-server/joint-trajectory/v1/docs/DENSE-JOINT-TRAJECTORY-V1.md).
// It is a reader for inspection, ported rule for rule from the Go twin
// (offline-programming/v1/internal/densejointtraj): it must reject exactly
// what the daemon's ingest rejects, in the same order and vocabulary, and
// agree on the content digest. The reference fixture in
// motion-server/joint-trajectory/v1/testdata is the arbiter; if this file
// and the fixture disagree, this file is wrong.
import { createHash } from "node:crypto";

export const SCHEMA = "robot.v4.dense-joint-trajectory.v1";
const DIGEST_DOMAIN = "robot.v4.dense-joint-trajectory.v1\x00";
const SAMPLE_ENCODING = "f64-be-aos.v1";
export const AXIS_COUNT = 9;
export const RECORD_BYTES = 8 + 8 * AXIS_COUNT + 8 * AXIS_COUNT + 1;
const MAX_TOTAL_SAMPLES = 250_000;
const BOUNDARY_QD_EPS_RAD_S = 1e-6;
const BOUNDARY_Q_EPS_RAD = 1e-3;
const TIME_GRID_EPS_S = 1e-9;
const Q_STEP_SLACK = 1.25;
const FLAG_TORCH_ON = 0x01;

export interface SegmentMeta {
  index: number;
  kind: string;
  source_id: string;
  sample_count: number;
  duration_s: number;
  block: { byte_offset: number; byte_length: number; sha256: string };
}

export interface Header {
  schema: string;
  plan_id: string;
  program_id: string;
  program_digest: string;
  trajectory_digest: string;
  manifest_revision: number;
  plan_revision: number;
  created_at: string;
  axes: { axis_count: number; axis_mask: number; position_unit: string; velocity_unit: string };
  sampling: { dt_s: number; total_sample_count: number; total_duration_s: number };
  limits: { max_abs_qd_rad_s: number[] };
  segments: SegmentMeta[];
  sample_encoding: { encoding: string; record_bytes: number };
  robot_cell?: Record<string, unknown>;
  start_state?: Record<string, unknown>;
  process_markers?: Array<Record<string, unknown>>;
  process_markers_sha256?: string;
}

export interface Sample {
  t: number;
  q: number[];
  qd: number[];
  torch: boolean;
}

export interface Segment {
  kind: string;
  sourceId: string;
  samples: Sample[];
  block: Uint8Array;
}

export class Rejection extends Error {
  constructor(
    readonly reason: string,
    readonly segmentIndex: number,
    readonly detail: string,
  ) {
    super(segmentIndex >= 0 ? `${reason}: segment ${segmentIndex}: ${detail}` : `${reason}: ${detail}`);
  }
}

function sha256Hex(bytes: Uint8Array): string {
  return createHash("sha256").update(bytes).digest("hex");
}

function decodeBlock(raw: Uint8Array, segment: number): Sample[] {
  const view = new DataView(raw.buffer, raw.byteOffset, raw.byteLength);
  const samples: Sample[] = [];
  for (let offset = 0; offset < raw.byteLength; offset += RECORD_BYTES) {
    const q: number[] = [];
    const qd: number[] = [];
    for (let a = 0; a < AXIS_COUNT; a++) {
      q.push(view.getFloat64(offset + 8 + 8 * a, false));
      qd.push(view.getFloat64(offset + 8 + 8 * AXIS_COUNT + 8 * a, false));
    }
    const flags = view.getUint8(offset + RECORD_BYTES - 1);
    if ((flags & ~FLAG_TORCH_ON) !== 0) {
      throw new Rejection("reserved_flags_set", segment, `sample ${samples.length}: reserved flag bits 0x${flags.toString(16).padStart(2, "0")} set`);
    }
    samples.push({ t: view.getFloat64(offset, false), q, qd, torch: (flags & FLAG_TORCH_ON) !== 0 });
  }
  return samples;
}

/** The content digest (DENSE-JOINT-TRAJECTORY-V1.md "Content digest"); block bytes are hashed verbatim. */
export function trajectoryDigest(axisMask: number, dtS: number, limits: number[], segments: Segment[]): string {
  const hash = createHash("sha256");
  const u64 = (value: number | bigint) => {
    const raw = Buffer.alloc(8);
    raw.writeBigUInt64BE(BigInt(value));
    hash.update(raw);
  };
  const f64 = (value: number) => {
    const raw = Buffer.alloc(8);
    raw.writeDoubleBE(value);
    hash.update(raw);
  };
  const str = (value: string) => {
    const bytes = Buffer.from(value, "utf8");
    u64(bytes.length);
    hash.update(bytes);
  };
  hash.update(Buffer.from(DIGEST_DOMAIN, "utf8"));
  u64(AXIS_COUNT);
  u64(axisMask >>> 0);
  f64(dtS);
  u64(segments.length);
  for (const segment of segments) {
    str(segment.kind);
    str(segment.sourceId);
    u64(segment.samples.length);
    hash.update(segment.block);
  }
  u64(AXIS_COUNT);
  for (const limit of limits) f64(limit);
  return `sha256:${hash.digest("hex")}`;
}

function validateMotion(header: Header, segments: Segment[]): void {
  const dt = header.sampling.dt_s;
  const limits = header.limits.max_abs_qd_rad_s;
  let previousLastQ: number[] | null = null;
  segments.forEach((segment, i) => {
    const samples = segment.samples;
    samples.forEach((sample, s) => {
      if (!Number.isFinite(sample.t)) throw new Rejection("nonfinite_sample", i, `sample ${s} time`);
      for (let a = 0; a < AXIS_COUNT; a++) {
        if (!Number.isFinite(sample.q[a]!) || !Number.isFinite(sample.qd[a]!)) {
          throw new Rejection("nonfinite_sample", i, `sample ${s} axis ${a}`);
        }
        if (Math.abs(sample.qd[a]!) > limits[a]!) {
          throw new Rejection("qd_limit_exceeded", i, `sample ${s} axis ${a}: |${sample.qd[a]!.toFixed(6)}| rad/s > limit ${limits[a]!.toFixed(6)} rad/s`);
        }
      }
      if (sample.torch && segment.kind !== "weld") {
        throw new Rejection("torch_outside_weld", i, `torch bit set at sample ${s} of a ${segment.kind} segment`);
      }
    });
    if (samples[0]!.t !== 0) throw new Rejection("time_grid_invalid", i, `t_s[0] is ${samples[0]!.t}, not 0.0`);
    for (let s = 1; s < samples.length; s++) {
      const step = samples[s]!.t - samples[s - 1]!.t;
      if (Math.abs(step - dt) > TIME_GRID_EPS_S) {
        throw new Rejection("time_grid_invalid", i, `t step ${step.toFixed(12)} s at sample ${s} vs dt_s ${dt} s`);
      }
      for (let a = 0; a < AXIS_COUNT; a++) {
        const dq = Math.abs(samples[s]!.q[a]! - samples[s - 1]!.q[a]!);
        const allowed = Q_STEP_SLACK * limits[a]! * dt;
        if (dq > allowed) throw new Rejection("q_step_exceeded", i, `sample ${s} axis ${a}: |dq| ${dq.toFixed(6)} rad > ${allowed.toFixed(6)} rad`);
      }
    }
    for (const boundary of [0, samples.length - 1]) {
      for (let a = 0; a < AXIS_COUNT; a++) {
        if (Math.abs(samples[boundary]!.qd[a]!) > BOUNDARY_QD_EPS_RAD_S) {
          throw new Rejection("boundary_qd_nonzero", i, `sample ${boundary} axis ${a}: |${samples[boundary]!.qd[a]!.toFixed(9)}| rad/s > ${BOUNDARY_QD_EPS_RAD_S} rad/s`);
        }
      }
    }
    if (previousLastQ) {
      for (let a = 0; a < AXIS_COUNT; a++) {
        const gap = Math.abs(samples[0]!.q[a]! - previousLastQ[a]!);
        if (gap > BOUNDARY_Q_EPS_RAD) {
          throw new Rejection("boundary_q_discontinuity", i, `axis ${a}: ${gap.toFixed(6)} rad gap from previous segment > ${BOUNDARY_Q_EPS_RAD} rad`);
        }
      }
    }
    const last = samples[samples.length - 1]!;
    const meta = header.segments[i]!;
    if (Math.abs(last.t - meta.duration_s) > TIME_GRID_EPS_S) {
      throw new Rejection("duration_mismatch", i, `duration_s ${meta.duration_s} vs last t_s ${last.t}`);
    }
    previousLastQ = last.q;
  });
}

/** Parses and validates a blob with every rule the daemon's ingest enforces. */
export function decode(blob: Uint8Array): { header: Header; segments: Segment[] } {
  if (blob.byteLength < 4) throw new Rejection("header_truncated", -1, `${blob.byteLength} bytes`);
  const view = new DataView(blob.buffer, blob.byteOffset, blob.byteLength);
  const headerLen = view.getUint32(0, false);
  if (4 + headerLen > blob.byteLength) throw new Rejection("header_truncated", -1, `header_len ${headerLen} exceeds blob ${blob.byteLength}`);
  let header: Header;
  try {
    header = JSON.parse(Buffer.from(blob.subarray(4, 4 + headerLen)).toString("utf8")) as Header;
  } catch (error) {
    throw new Rejection("header_json_invalid", -1, error instanceof Error ? error.message : String(error));
  }
  if (header.schema !== SCHEMA) throw new Rejection("schema_mismatch", -1, `schema ${JSON.stringify(header.schema)}`);
  if (header.axes?.axis_count !== AXIS_COUNT) throw new Rejection("axis_count_mismatch", -1, `axis_count ${header.axes?.axis_count} != ${AXIS_COUNT}`);
  if (header.sample_encoding?.encoding !== SAMPLE_ENCODING || header.sample_encoding?.record_bytes !== RECORD_BYTES) {
    throw new Rejection("sample_encoding_mismatch", -1, `${JSON.stringify(header.sample_encoding?.encoding)} record_bytes ${header.sample_encoding?.record_bytes}`);
  }
  const limits = header.limits?.max_abs_qd_rad_s;
  if (!Array.isArray(limits) || limits.length !== AXIS_COUNT) throw new Rejection("limits_invalid", -1, `max_abs_qd_rad_s has ${limits?.length ?? 0} values`);
  for (const limit of limits) if (!(limit > 0) || !Number.isFinite(limit)) throw new Rejection("limits_invalid", -1, `limit ${limit}`);
  if (!(header.sampling?.dt_s > 0)) throw new Rejection("time_grid_invalid", -1, `dt_s ${header.sampling?.dt_s}`);
  if (!Array.isArray(header.segments) || header.segments.length === 0) throw new Rejection("segments_empty", -1, "no segments");

  const body = blob.subarray(4 + headerLen);
  const segments: Segment[] = [];
  let offset = 0;
  let total = 0;
  header.segments.forEach((meta, i) => {
    meta.block ??= { byte_offset: 0, byte_length: 0, sha256: "" };
    if (meta.index !== i) throw new Rejection("segment_index_out_of_order", i, `index ${meta.index}`);
    if (!["freespace", "weld", "dwell"].includes(meta.kind)) throw new Rejection("kind_invalid", i, `kind ${JSON.stringify(meta.kind)}`);
    if (meta.sample_count < 2) throw new Rejection("segment_too_short", i, `${meta.sample_count} samples; a segment needs at least 2`);
    if (meta.block.byte_offset !== offset) throw new Rejection("block_layout_invalid", i, `byte_offset ${meta.block.byte_offset}, expected ${offset}`);
    if (meta.block.byte_length !== meta.sample_count * RECORD_BYTES) {
      throw new Rejection("block_layout_invalid", i, `byte_length ${meta.block.byte_length} != sample_count ${meta.sample_count} * ${RECORD_BYTES}`);
    }
    if (offset + meta.block.byte_length > body.byteLength) {
      throw new Rejection("blob_length_mismatch", i, `block ends at ${offset + meta.block.byte_length}, body is ${body.byteLength}`);
    }
    const raw = body.subarray(offset, offset + meta.block.byte_length);
    const computed = `sha256:${sha256Hex(raw)}`;
    if (computed !== meta.block.sha256) throw new Rejection("block_sha256_mismatch", i, `stored ${meta.block.sha256} computed ${computed}`);
    segments.push({ kind: meta.kind, sourceId: meta.source_id, samples: decodeBlock(raw, i), block: raw });
    offset += meta.block.byte_length;
    total += meta.sample_count;
  });
  if (offset !== body.byteLength) throw new Rejection("blob_length_mismatch", -1, `blocks end at ${offset}, body is ${body.byteLength}`);
  if (header.sampling.total_sample_count !== total) {
    throw new Rejection("total_sample_count_mismatch", -1, `header ${header.sampling.total_sample_count}, blocks ${total}`);
  }
  if (total > MAX_TOTAL_SAMPLES) throw new Rejection("sample_count_overflow", -1, `${total} samples > ${MAX_TOTAL_SAMPLES}`);
  validateMotion(header, segments);
  const digest = trajectoryDigest(header.axes.axis_mask, header.sampling.dt_s, limits, segments);
  if (digest !== header.trajectory_digest) throw new Rejection("trajectory_digest_mismatch", -1, `stored ${header.trajectory_digest} computed ${digest}`);
  return { header, segments };
}

export interface AxisSummary {
  axis: string;
  in_mask: boolean;
  min_q_rad: number;
  max_q_rad: number;
  max_abs_qd_rad_s: number;
  limit_qd_rad_s: number;
  /** Peak |qd| as a fraction of the header's executor ceiling for the axis. */
  qd_utilisation: number;
}

export interface TrajectorySummary {
  valid: boolean;
  rejection: { reason: string; segment_index: number | null; detail: string } | null;
  bytes: number;
  identity: Pick<Header, "plan_id" | "program_id" | "program_digest" | "trajectory_digest" | "manifest_revision" | "plan_revision" | "created_at"> | null;
  dt_s: number | null;
  total_samples: number | null;
  total_duration_s: number | null;
  axis_mask: number | null;
  segments: Array<{ index: number; kind: string; source_id: string; samples: number; duration_s: number; torch_samples: number }>;
  axes: AxisSummary[];
  robot_cell: Record<string, unknown> | null;
  start_state: Record<string, unknown> | null;
  process_markers: number;
}

/** Decodes, validates and summarises a blob; a rejection is reported, not thrown. */
export function summarize(blob: Uint8Array): TrajectorySummary {
  const summary: TrajectorySummary = {
    valid: false,
    rejection: null,
    bytes: blob.byteLength,
    identity: null,
    dt_s: null,
    total_samples: null,
    total_duration_s: null,
    axis_mask: null,
    segments: [],
    axes: [],
    robot_cell: null,
    start_state: null,
    process_markers: 0,
  };
  let decoded: { header: Header; segments: Segment[] };
  try {
    decoded = decode(blob);
  } catch (error) {
    if (!(error instanceof Rejection)) throw error;
    summary.rejection = { reason: error.reason, segment_index: error.segmentIndex >= 0 ? error.segmentIndex : null, detail: error.detail };
    return summary;
  }
  const { header, segments } = decoded;
  summary.valid = true;
  summary.identity = {
    plan_id: header.plan_id,
    program_id: header.program_id,
    program_digest: header.program_digest,
    trajectory_digest: header.trajectory_digest,
    manifest_revision: header.manifest_revision,
    plan_revision: header.plan_revision,
    created_at: header.created_at,
  };
  summary.dt_s = header.sampling.dt_s;
  summary.total_samples = header.sampling.total_sample_count;
  summary.total_duration_s = header.sampling.total_duration_s;
  summary.axis_mask = header.axes.axis_mask;
  summary.robot_cell = header.robot_cell ?? null;
  summary.start_state = header.start_state ?? null;
  summary.process_markers = header.process_markers?.length ?? 0;
  summary.segments = segments.map((segment, index) => ({
    index,
    kind: segment.kind,
    source_id: segment.sourceId,
    samples: segment.samples.length,
    duration_s: header.segments[index]!.duration_s,
    torch_samples: segment.samples.filter((sample) => sample.torch).length,
  }));
  for (let a = 0; a < AXIS_COUNT; a++) {
    let min = Infinity;
    let max = -Infinity;
    let peak = 0;
    for (const segment of segments) {
      for (const sample of segment.samples) {
        min = Math.min(min, sample.q[a]!);
        max = Math.max(max, sample.q[a]!);
        peak = Math.max(peak, Math.abs(sample.qd[a]!));
      }
    }
    const limit = header.limits.max_abs_qd_rad_s[a]!;
    summary.axes.push({
      axis: `J${a + 1}`,
      in_mask: (header.axes.axis_mask & (1 << a)) !== 0,
      min_q_rad: min,
      max_q_rad: max,
      max_abs_qd_rad_s: peak,
      limit_qd_rad_s: limit,
      qd_utilisation: peak / limit,
    });
  }
  return summary;
}
