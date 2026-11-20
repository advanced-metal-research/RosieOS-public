import type { DenseCapabilities, DenseTelemetryWindow } from "./denseClient.ts";
import type { SimulationSample } from "../simulation/contracts.ts";
import type { MotionPathPoint } from "../viewer/geometry.ts";

export interface ExecutionPathSamples {
  actual: SimulationSample[][];
  commanded: SimulationSample[][];
  /** Same shape as the path runs; false across missing or unsubmitted references. */
  comparable: boolean[][];
  sequences: number[][];
  omitted: number;
  trajectoryId: number;
}

/** Preserve the measured coordinate frame. Never zero a capture to its first
 * sample or join missing data; either would fabricate a TCP path. */
export function executionPathSamples(
  window: DenseTelemetryWindow, capabilities: DenseCapabilities | null,
  modelId: string, jointOrder: readonly string[],
): ExecutionPathSamples {
  if (!window || !Array.isArray(window.records) || window.records.length > 100_000 || window.records.length < 2 ||
      !Number.isFinite(window.cycle_period_ns) || window.cycle_period_ns <= 0) throw new Error("Invalid or oversized execution capture");
  const manifest = capabilities?.rtCore;
  if (!manifest || !window.configuration_sha256 || window.configuration_sha256 !== manifest.configurationSHA256 || !window.daemon_incarnation) {
    throw new Error("The capture's machine configuration does not match the connected robot");
  }
  if (window.stride !== 1) throw new Error("TCP comparison requires every recorded cycle; capture at 1 kHz");
  const axes = jointOrder.map(name => manifest.axes.find(axis => axis.id === name));
  if (axes.some(axis => !axis || axis.position_unit !== "rad" || !Number.isFinite(axis.counts_per_unit) || axis.counts_per_unit <= 0 || ![-1, 1].includes(axis.sign))) {
    throw new Error("The capture must describe every robot joint in radians");
  }
  const indices = axes.map(axis => axis!.index);
  if (new Set(indices).size !== indices.length || indices.some(i => !Number.isInteger(i) || i < 0 || i >= window.axis_count)) throw new Error("Ambiguous capture joint mapping");
  for (const [i, record] of window.records.entries()) {
    if (!Array.isArray(record.ax) || !Number.isSafeInteger(record.seq) || !Number.isSafeInteger(record.t_ns) ||
        (i > 0 && (record.seq <= window.records[i - 1]!.seq || record.t_ns <= window.records[i - 1]!.t_ns))) {
      throw new Error("Capture sequences and timestamps must be strictly increasing");
    }
  }
  const trajectoryId = [...window.records].reverse().find(r => r.mode === 2 && (r.trajectory_id ?? 0) > 0)?.trajectory_id;
  if (!trajectoryId) throw new Error("No executed plan is in this capture window");
  const records = window.records.filter(r => r.mode === 2 && r.trajectory_id === trajectoryId);
  const first = records[0]!;
  if (!first.configuration_epoch || !first.home_epoch) throw new Error("Capture lacks coordinate identity; refresh the stack and capture again");
  if (records.some(r => r.configuration_epoch !== first.configuration_epoch || r.home_epoch !== first.home_epoch)) {
    throw new Error("Home or configuration changed during the capture; capture one coordinate frame at a time");
  }
  const result: ExecutionPathSamples = { actual: [], commanded: [], comparable: [], sequences: [], omitted: 0, trajectoryId };
  let previous: typeof first | undefined;
  let run = -1;
  for (const record of records) {
    const valid = indices.every(i => {
      const axis = record.ax[i];
      return axis?.coordinate_ok && Number.isSafeInteger(axis.coordinate_counts) && Number.isSafeInteger(axis.home_offset) && Number.isSafeInteger(axis.tp);
    });
    if (!valid) { result.omitted++; previous = undefined; continue; }
    const contiguous = previous && record.seq === previous.seq + 1 && record.t_ns > previous.t_ns &&
      record.t_ns - previous.t_ns <= window.cycle_period_ns * 1.5;
    if (!contiguous) {
      run++;
      result.actual.push([]); result.commanded.push([]); result.comparable.push([]); result.sequences.push([]);
    }
    const measured: number[] = [], commanded: number[] = [];
    for (const [j, index] of indices.entries()) {
      const description = axes[j]!;
      const axis = record.ax[index]!;
      // The comparison uses the preceding successfully submitted command, as
      // the feedback precedes the current cycle's output. No fitted lag shift.
      const reference = contiguous ? previous!.ax[index]! : axis;
      const target = reference.tp;
      measured.push((axis.coordinate_counts! + axis.home_offset!) * description.sign / description.counts_per_unit);
      commanded.push((target + axis.home_offset!) * description.sign / description.counts_per_unit);
    }
    const sample = (jointsRad: number[], source: "live" | "preview"): SimulationSample => ({
      modelId, jointOrder: [...jointOrder], jointsRad, source,
      timeS: (record.t_ns - first.t_ns) / 1e9, motion: "travel",
    });
    result.actual[run]!.push(sample(measured, "live"));
    result.commanded[run]!.push(sample(commanded, "preview"));
    result.comparable[run]!.push(Boolean(contiguous && record.state === 3 && previous!.state === 3 &&
      indices.every(index => ((previous!.submitted_mask ?? 0) & (1 << index)) !== 0)));
    result.sequences[run]!.push(record.seq);
    previous = record;
  }
  if (result.actual.flat().length < 2) throw new Error("Not enough valid homed joint feedback to reconstruct the TCP path");
  return result;
}

export interface TcpErrorPoint { timeS: number; errorMM: number | null }
export function executionTcpErrors(samples: ExecutionPathSamples, actual: MotionPathPoint[][], commanded: MotionPathPoint[][]) {
  const points: TcpErrorPoint[] = [];
  let sum = 0, peakMM = 0, count = 0;
  for (const [run, poses] of samples.actual.entries()) {
    if (actual[run]?.length !== poses.length || commanded[run]?.length !== poses.length) throw new Error("TCP trace is incomplete");
    for (const [i, pose] of poses.entries()) {
      const a = actual[run]![i]!.positionM, c = commanded[run]![i]!.positionM;
      const errorMM = samples.comparable[run]![i] ? Math.hypot(...a.map((v, j) => v - c[j]!)) * 1000 : null;
      if (errorMM !== null) { peakMM = Math.max(peakMM, errorMM); sum += errorMM * errorMM; count++; }
      points.push({ timeS: pose.timeS, errorMM });
    }
    points.push({ timeS: poses.at(-1)!.timeS, errorMM: null });
  }
  return { points, peakMM, rmsMM: count ? Math.sqrt(sum / count) : null, count };
}
