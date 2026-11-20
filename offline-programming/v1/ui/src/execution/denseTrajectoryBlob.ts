/**
 * Reading `robot.v4.dense-joint-trajectory.v1` in the browser.
 *
 * The fourth hand-rolled implementation of this layout, after the Python
 * encoder (`weldplan/dense_joint_trajectory.py`), the Go twin
 * (`internal/densejointtraj`) and the C++ daemon. That is a cost, and the
 * encoder says why it is paid: the packed block bytes are also the digest
 * input, so a serialization library whose output is not canonical would force
 * a second layout for the digest anyway.
 *
 * What keeps a fourth copy honest is the same arbiter the Go twin answers to --
 * `motion-server/joint-trajectory/v1/testdata/dense_joint_trajectory_reference.rdt` and the
 * expected values beside it. If this file and the fixture disagree, this file
 * is wrong.
 *
 * This one only READS, which bounds the damage: a reader that has drifted
 * fails to parse or reports the wrong count, and both are visible. It never
 * writes bytes anyone plays.
 *
 * Layout:
 *
 *     [u32 BE header_len][UTF-8 JSON header][block 0]..[block S-1]
 *
 * One block per motion segment, in segment order, each a run of 153-byte
 * big-endian records:
 *
 *     f64 t_s, f64 q_rad[9], f64 qd_rad_s[9], u8 flags   (bit0 = torch_on)
 *
 * Every block keeps its OWN clock: `t_s` starts at 0.0 in each one. Gluing
 * them onto a single timeline is the mistake the encoder names explicitly, and
 * a player walks the segments in order rather than sorting samples by time.
 */

/** Bytes per sample: a time, nine joints, nine rates, one flag byte. */
export const RECORD_BYTES = 8 + 9 * 8 + 9 * 8 + 1;

/** Joint axes per sample. The arm and the positioner, in the cell's order. */
export const AXES = 9;

export interface DenseSample {
  /** Seconds from the START OF THIS SEGMENT, not of the trajectory. */
  tS: number;
  qRad: Float64Array;
  qdRadS: Float64Array;
  torchOn: boolean;
}

export interface DenseSegment {
  index: number;
  kind: string;
  sourceId: string;
  samples: DenseSample[];
}

export interface DenseTrajectory {
  schema: string;
  trajectoryDigest: string;
  programDigest?: string;
  startState?: { coordinate_identity: string; observation_identity: string; joint_names: string[]; joint_values_rad: number[] };
  robot?: { modelId: string; robotDescriptionSha256: string; machinePlanningCalibrationSha256: string; machinePlanningCalibrationSource?: string };
  dtS: number;
  segments: DenseSegment[];
  totalSampleCount: number;
}

export class DenseBlobError extends Error {}

/** Decode one blob. Throws rather than returning a partial trajectory: half a
 *  motion is not a shorter motion, and a player handed one would run the arm
 *  somewhere nobody planned. */
export function decodeDenseTrajectory(blob: Uint8Array): DenseTrajectory {
  if (blob.byteLength < 4) throw new DenseBlobError("too short to hold a header length");
  const view = new DataView(blob.buffer, blob.byteOffset, blob.byteLength);
  const headerLength = view.getUint32(0, false);
  const headerEnd = 4 + headerLength;
  if (headerLength === 0 || headerEnd > blob.byteLength) {
    throw new DenseBlobError(`header length ${headerLength} does not fit in ${blob.byteLength} bytes`);
  }

  let header: Record<string, unknown>;
  try {
    header = JSON.parse(new TextDecoder().decode(blob.subarray(4, headerEnd))) as Record<string, unknown>;
  } catch (error) {
    throw new DenseBlobError(`header is not JSON: ${String(error)}`);
  }

  const declared = Array.isArray(header.segments) ? header.segments : [];
  const segments: DenseSegment[] = [];
  let offset = headerEnd;
  let total = 0;

  declared.forEach((raw, index) => {
    const entry = (raw ?? {}) as Record<string, unknown>;
    const count = Number(entry.sample_count ?? 0);
    if (!Number.isInteger(count) || count < 0) {
      throw new DenseBlobError(`segment ${index} declares ${entry.sample_count} samples`);
    }
    const end = offset + count * RECORD_BYTES;
    if (end > blob.byteLength) {
      throw new DenseBlobError(
        `segment ${index} wants ${count} samples and the blob ends ${blob.byteLength - offset} bytes in`,
      );
    }
    const samples: DenseSample[] = [];
    for (let sample = 0; sample < count; sample += 1) {
      const at = offset + sample * RECORD_BYTES;
      const qRad = new Float64Array(AXES);
      const qdRadS = new Float64Array(AXES);
      for (let axis = 0; axis < AXES; axis += 1) {
        qRad[axis] = view.getFloat64(at + 8 + axis * 8, false);
        qdRadS[axis] = view.getFloat64(at + 8 + AXES * 8 + axis * 8, false);
      }
      samples.push({
        tS: view.getFloat64(at, false),
        qRad,
        qdRadS,
        // bit0. The rest are reserved and zero; reading only bit0 keeps a
        // future flag from arriving as a torch that will not switch off.
        torchOn: (view.getUint8(at + RECORD_BYTES - 1) & 1) === 1,
      });
    }
    segments.push({
      index: Number(entry.index ?? index),
      kind: String(entry.kind ?? "freespace"),
      sourceId: String(entry.source_id ?? ""),
      samples,
    });
    offset = end;
    total += count;
  });

  if (offset !== blob.byteLength) {
    throw new DenseBlobError(
      `${blob.byteLength - offset} bytes past the last segment: the header and the blocks disagree`,
    );
  }
  const robot = header.robot_cell as Record<string, unknown> | undefined;
  return {
    schema: String(header.schema ?? ""),
    trajectoryDigest: String(header.trajectory_digest ?? ""),
    programDigest: String(header.program_digest ?? ""),
    ...(header.start_state ? { startState: header.start_state as DenseTrajectory["startState"] } : {}),
    robot: robot ? {
      modelId: String(robot.model_id ?? ""),
      robotDescriptionSha256: String(robot.robot_description_sha256 ?? ""),
      machinePlanningCalibrationSha256: String(robot.machine_planning_calibration_sha256 ?? ""),
      machinePlanningCalibrationSource: String(robot.machine_planning_calibration_source ?? ""),
    } : undefined,
    dtS: Number(header.dt_s ?? 0),
    segments,
    totalSampleCount: total,
  };
}

/** The trajectory as one list to step through, in segment order.
 *
 *  `tS` is rewritten to run from zero across the whole thing, which is a
 *  PLAYBACK convenience and not the contract: each segment's own clock is
 *  what the daemon honours, and a segment boundary is a stop rather than a
 *  join. The elapsed time here is for a scrubber, never for anything that
 *  commands motion.
 */
export function flattenForPlayback(trajectory: DenseTrajectory): {
  elapsedS: number;
  segmentIndex: number;
  sample: DenseSample;
}[] {
  const out: { elapsedS: number; segmentIndex: number; sample: DenseSample }[] = [];
  let elapsed = 0;
  for (const segment of trajectory.segments) {
    for (const sample of segment.samples) {
      out.push({ elapsedS: elapsed + sample.tS, segmentIndex: segment.index, sample });
    }
    const last = segment.samples[segment.samples.length - 1];
    if (last) elapsed += last.tS;
  }
  return out;
}
