import assert from "node:assert/strict";
import { test } from "node:test";
import {
  TELEMETRY_PALETTE,
  TELEMETRY_QUANTITIES,
  PreviewTrace,
  TelemetryRecorder,
  formatQuantity,
  niceRange,
  sampleFromStatus,
  samplesFromWindow,
  seriesFor,
  type TelemetrySample,
} from "./telemetryStrip.ts";
import type { DenseCapabilities, DenseStatus, DenseTelemetryWindow } from "./denseClient.ts";

const COUNTS_PER_RAD = 2_086_075.67;

function status(timeNs: number, axes: Partial<{ position: number; velocity: number; fe: number; torque: number; valid: boolean }>[]): DenseStatus {
  return {
    backend: "rt_core",
    rtCore: {
      timeNs,
      axes: axes.map(a => ({
        position: a.position ?? 0, target: a.position ?? 0, velocity: null, homeValid: true, readiness: "ready", statusword: 0, positionCounts: 0,
        velocityActualCountsPerS: a.velocity ?? 0, followingErrorCounts: a.fe ?? 0, torqueRaw: a.torque ?? 0,
        velocityActualValid: a.valid ?? true, followingErrorValid: a.valid ?? true,
      })),
    },
  } as unknown as DenseStatus;
}

const capabilities = {
  backend: "rt_core",
  rtCore: { axes: [{ id: "J1", position_unit: "rad", counts_per_unit: COUNTS_PER_RAD, sign: -1 }, { id: "J2", position_unit: "rad", counts_per_unit: COUNTS_PER_RAD, sign: 1 }] },
} as unknown as DenseCapabilities;

test("a status becomes one sample in axis units: counts divided by counts_per_unit with the axis sign, torque as reported", () => {
  const sample = sampleFromStatus(status(5_000_000_000, [{ position: 0.25, velocity: -41_506, fe: -741, torque: -10 }, { position: -0.1, velocity: 20_860.7567, fe: 2086.07567, torque: 7 }]), capabilities);
  assert.ok(sample);
  assert.equal(sample.tMs, 5000);
  // J1 has sign -1: a negative count velocity is a positive axis velocity.
  assert.ok(Math.abs(sample.axes[0]!.velocity! - 41_506 / COUNTS_PER_RAD) < 1e-9);
  assert.ok(Math.abs(sample.axes[0]!.followingError! - 741 / COUNTS_PER_RAD) < 1e-9);
  assert.equal(sample.axes[0]!.torque, -10);
  assert.ok(Math.abs(sample.axes[1]!.velocity! - 0.01) < 1e-9);
  assert.ok(Math.abs(sample.axes[1]!.followingError! - 0.001) < 1e-9);
  assert.equal(sample.axes[1]!.position, -0.1);
});

test("no description, no sample: a count the description cannot place in a unit is not plotted", () => {
  assert.equal(sampleFromStatus(status(1_000_000, [{ velocity: 100 }]), null), null);
  const partial = { backend: "rt_core", rtCore: { axes: [{ id: "J1", position_unit: "rad", counts_per_unit: 0, sign: 1 }] } } as unknown as DenseCapabilities;
  const sample = sampleFromStatus(status(1_000_000, [{ position: 0.5, velocity: 100, fe: 3, torque: 4 }]), partial);
  assert.ok(sample);
  assert.equal(sample.axes[0]!.velocity, null);
  assert.equal(sample.axes[0]!.followingError, null);
  assert.equal(sample.axes[0]!.position, 0.5);
  assert.equal(sample.axes[0]!.torque, 4);
});

test("an invalid drive measurement is null, not zero", () => {
  const sample = sampleFromStatus(status(1_000_000, [{ velocity: 100, fe: 3, valid: false }]), capabilities);
  assert.equal(sample!.axes[0]!.velocity, null);
  assert.equal(sample!.axes[0]!.followingError, null);
});

function sampleAt(tMs: number, velocity: number): TelemetrySample {
  return { tMs, axes: [{ position: 0, target: 0, velocity, followingError: 0, torque: 0 }] };
}

test("the recorder keeps a time-bounded history, drops repeats, and restarts after a daemon clock reset", () => {
  const recorder = new TelemetryRecorder(1000);
  for (let t = 0; t <= 2000; t += 100) recorder.push(sampleAt(t, t));
  assert.equal(recorder.length, 11, "one second of samples at 10 Hz plus the newest");
  assert.equal(recorder.window(500).length, 6);
  assert.equal(recorder.push(sampleAt(2000, 0)), false, "the same status polled twice is one sample");
  assert.equal(recorder.push(sampleAt(1900, 0)), false, "a slightly older status is dropped, not reordered");
  assert.equal(recorder.push(sampleAt(50, 0)), true, "time far behind the history means a restarted daemon");
  assert.equal(recorder.length, 1);
  assert.equal(new TelemetryRecorder().rateHz(10_000), null, "no rate from fewer than two samples");
});

test("the rate is read from the timestamps, not the poll timer", () => {
  const recorder = new TelemetryRecorder();
  for (let t = 0; t < 1000; t += 25) recorder.push(sampleAt(t, 0));
  assert.equal(Math.round(recorder.rateHz(10_000)!), 40);
});

test("acceleration is the central difference of the drive velocity and is absent at the ends", () => {
  const samples = [sampleAt(0, 0), sampleAt(100, 1), sampleAt(200, 3), sampleAt(300, 6)];
  const a = seriesFor(samples, 0, "acceleration");
  assert.deepEqual(a, [null, (3 - 0) / 0.2, (6 - 1) / 0.2, null]);
  assert.deepEqual(seriesFor(samples, 0, "velocity"), [0, 1, 3, 6]);
  assert.deepEqual(seriesFor(samples, 1, "velocity"), [null, null, null, null], "an axis the sample lacks is null throughout");
});

test("tick ranges round out to 1, 2 or 5 times a power of ten", () => {
  assert.deepEqual(niceRange(-0.37, 1.9), { min: -0.5, max: 2, step: 0.5 });
  assert.deepEqual(niceRange(3, 3), { min: 2, max: 4, step: 0.5 });
  assert.deepEqual(niceRange(0, 123), { min: 0, max: 150, step: 50 });
});

test("display units: degrees for angles, millidegrees for following error, per-mille torque as an integer", () => {
  const spec = (q: string) => TELEMETRY_QUANTITIES.find(s => s.quantity === q)!;
  assert.equal(formatQuantity(Math.PI / 6, spec("position")), "30.00");
  assert.equal(formatQuantity(741 / COUNTS_PER_RAD, spec("following_error")), "20.4");
  assert.equal(formatQuantity(-10.4, spec("torque")), "-10");
  assert.equal(formatQuantity(null, spec("velocity")), "—");
  assert.equal(TELEMETRY_PALETTE.length, 6, "one fixed hue per axis of a six-axis arm");
});

test("a ring window becomes samples with positions relative to its first record and the same unit conversion as the live view", () => {
  const axis = (p: number, v: number, fe: number, tq: number, ok = true) => ({ p, tp: p + 10, v, v_ok: ok, tv: 0, tv_ok: false, fe, fe_ok: ok, tq, tq_ok: ok });
  const window: DenseTelemetryWindow = {
    cycle_period_ns: 1_000_000, axis_count: 2, stride: 1, seconds: 0.003, first_sequence: 100, last_sequence: 102, head_estimate: 102, fetched_records: 3, fetch_ms: 12,
    records: [
      { seq: 100, t_ns: 5_000_000_000, work_ns: 0, jitter_ns: 0, armed: true, jog_open: false, state: 0, mode: 0, fault_mask: 0, exec_fault: 0, ax: [axis(1_000_000, -41_506, -741, -10), axis(500, 2086.07567, 0, 0, false)] },
      { seq: 101, t_ns: 5_001_000_000, work_ns: 0, jitter_ns: 0, armed: true, jog_open: false, state: 0, mode: 0, fault_mask: 0, exec_fault: 0, ax: [axis(1_000_000 - 2086.07567, -41_506, -741, -10), axis(500, 0, 0, 0)] },
      { seq: 102, t_ns: 5_002_000_000, work_ns: 0, jitter_ns: 0, armed: true, jog_open: false, state: 0, mode: 0, fault_mask: 0, exec_fault: 0, ax: [axis(1_000_000, 0, 0, 0), axis(500, 0, 0, 0)] },
    ],
  };
  const samples = samplesFromWindow(window, capabilities);
  assert.equal(samples.length, 3);
  assert.equal(samples[0]!.tMs, 5000);
  assert.equal(samples[0]!.axes[0]!.position, 0, "the first record is the origin");
  // J1 has sign -1: a fall of 2086 counts is a rise of a milliradian.
  assert.ok(Math.abs(samples[1]!.axes[0]!.position! - 0.001) < 1e-9);
  assert.ok(Math.abs(samples[1]!.axes[0]!.target! - (0.001 - 10 / COUNTS_PER_RAD)) < 1e-9);
  assert.ok(Math.abs(samples[0]!.axes[0]!.velocity! - 41_506 / COUNTS_PER_RAD) < 1e-9);
  assert.equal(samples[0]!.axes[0]!.torque, -10);
  assert.equal(samples[0]!.axes[1]!.velocity, null, "an invalid drive measurement is null");
  assert.equal(samples[0]!.axes[1]!.torque, null);
  assert.deepEqual(samplesFromWindow(window, null), [], "no description, no samples");
});

test("dense captures smooth the velocity before differencing; the live view does not", () => {
  // A ramp of 100 units/s² with quantisation noise of period three, so the
  // two-sample central difference sees it and a nine-sample mean cancels it.
  const noise = [0.05, -0.05, 0];
  const samples: TelemetrySample[] = [];
  for (let i = 0; i < 40; i++) samples.push(sampleAt(i, i * 0.1 + noise[i % 3]!));
  const raw = seriesFor(samples, 0, "acceleration");
  const smooth = seriesFor(samples, 0, "acceleration", 9);
  const mid = 20;
  // The raw central difference sees the quantisation as ±50 units/s² noise around the 100 units/s² ramp.
  assert.ok(Math.abs(raw[mid]! - 100) > 20, `raw ${raw[mid]}`);
  assert.ok(Math.abs(smooth[mid]! - 100) < 5, `smooth ${smooth[mid]}`);
  assert.equal(smooth[0], null);
  assert.equal(smooth.at(-1), null);
});

test("preview samples overlay the position series in the strip's display units and never enter the machine recorder", () => {
  const recorder = new TelemetryRecorder();
  recorder.push(sampleAt(1000, 0));
  recorder.push(sampleAt(1100, 0));
  const before = recorder.length;
  const preview = new PreviewTrace();
  const sample = (jointsRad: readonly number[]) => ({ jointOrder: ["J1", "J2"], jointsRad });
  preview.previewSample(sample([0.5, -0.25]), ["J1", "J2"], 1000);
  preview.previewSample(sample([-0.25, 0.5]), ["J1", "J2"], 1100);
  assert.equal(recorder.length, before);
  const spec = TELEMETRY_QUANTITIES.find(q => q.quantity === "position")!;
  assert.equal(spec.scale, 180 / Math.PI);
  assert.deepEqual(preview.series(0).map(v => v === null ? null : v * spec.scale), [0.5 * spec.scale, -0.25 * spec.scale]);
  assert.deepEqual(preview.series(1).map(v => v === null ? null : v * spec.scale), [-0.25 * spec.scale, 0.5 * spec.scale]);
  preview.previewSample(null, ["J1", "J2"], 1200);
  assert.equal(preview.active, false);
  assert.deepEqual(preview.series(0), []);
  assert.equal(recorder.length, before);
});

test("a preview sample against an axis with no matching id maps by index", () => {
  const preview = new PreviewTrace();
  preview.previewSample({ jointOrder: ["J1", "J2"], jointsRad: [0.5, -0.25] }, ["axis_a", "axis_b"], 1000);
  const spec = TELEMETRY_QUANTITIES.find(q => q.quantity === "position")!;
  assert.deepEqual(preview.series(0).map(v => v === null ? null : v * spec.scale), [0.5 * spec.scale]);
  assert.deepEqual(preview.series(1).map(v => v === null ? null : v * spec.scale), [-0.25 * spec.scale]);
});
