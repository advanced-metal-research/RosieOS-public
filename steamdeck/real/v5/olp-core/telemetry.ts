// The Telemetry page's plot, computed by OLP's telemetry code (the arithmetic
// and words telemetryStrip.ts draws in the browser): live samples from each
// status poll, a 1 kHz capture read from the machine's ring, the quantity and
// window tabs, the ranges, ticks, direct labels and the legend readouts. The
// Qt side paints the lines and text this returns and nothing else.
import { normalizeTelemetryWindow } from "../../../../offline-programming/v1/ui/src/execution/denseClient.ts";
import {
  TELEMETRY_CAPTURE_DEFAULT_SECONDS, TELEMETRY_CAPTURE_SECONDS, TELEMETRY_PALETTE, TELEMETRY_PALETTE_DARK, TELEMETRY_QUANTITIES, TELEMETRY_WINDOWS_MS,
  TelemetryRecorder, axisReadout, captureFailedText, captureNote, captureReadingText, formatQuantity, parseTelemetryView, plotRange, plotScales,
  plotTitle, quantitySpec, sampleFromStatus, samplesFromWindow, seriesFor, shownTelemetry, spreadLabels, telemetryMeta,
  telemetrySource, timeTicks, valueTicks, type TelemetryQuantity,
} from "../../../../offline-programming/v1/ui/src/execution/telemetrySeries.ts";
import { machineCapabilities, machineStatus } from "./machine.ts";

// The longest capture the pendant offers. Captures are decoded and plotted on
// the UI thread, where a freeze also holds up the controller's STOP (B, R5).
// Measured in Qt 5.15's engine, 9 axes: 3 s = 0.1 s to load and 30-44 ms a
// plot; 100 s = 5.6-9.1 s to load and 4-6.8 s for the first plot. 10 s is
// interpolated from those (about 0.3 s + 0.15 s), not measured. OLP's
// browser offers 30 s and 100 s as well.
const PENDANT_CAPTURE_MAX_SECONDS = 10;

const live = new TelemetryRecorder();
let capture: TelemetryRecorder | null = null;
let note: string | null = null;
let heldAtMs: number | null = null;

/** After each applied status poll: the poll's sample joins the live history. */
export function telemetrySample(): boolean {
  const sample = sampleFromStatus(machineStatus(), machineCapabilities());
  return sample ? live.push(sample) : false;
}

export function telemetryChoices() {
  return {
    quantities: TELEMETRY_QUANTITIES.map(spec => ({ quantity: spec.quantity, short: spec.short })),
    windowsMs: [...TELEMETRY_WINDOWS_MS],
    captureSeconds: TELEMETRY_CAPTURE_SECONDS.filter(seconds => seconds <= PENDANT_CAPTURE_MAX_SECONDS),
    captureDefaultSeconds: TELEMETRY_CAPTURE_DEFAULT_SECONDS,
  };
}

/** A stored view (quantity, window, hidden axes), or OLP's default for anything it cannot name. */
export function telemetryView(raw: string | null) {
  return parseTelemetryView(raw);
}

export function telemetryCaptureReading(seconds: number): string {
  return captureReadingText(seconds);
}

export function telemetryCaptureFailed(message: string): string {
  return captureFailedText(message);
}

/** A GET /telemetry/window reply becomes the shown capture; returns its note. */
export function telemetryCapture(payload: unknown): string {
  const window = normalizeTelemetryWindow(payload);
  const recorder = new TelemetryRecorder(Number.POSITIVE_INFINITY);
  for (const sample of samplesFromWindow(window, machineCapabilities())) recorder.push(sample);
  capture = recorder;
  capturedPlot = null;
  note = captureNote(window);
  heldAtMs = null;
  return note;
}

/** Back to the live window. */
export function telemetryLive(): void {
  capture = null;
  capturedPlot = null;
  note = null;
}

/** Hold stops the live plot at its newest sample while sampling continues. */
export function telemetryHold(on: boolean): void {
  heldAtMs = on ? live.latest()?.tMs ?? null : null;
}

export function telemetryForget(): void {
  live.clear();
  capture = null;
  capturedPlot = null;
  note = null;
  heldAtMs = null;
}

export interface PlotInput {
  /** OLP's dark plot palette, validated for its dark surface. */
  dark?: boolean;
  quantity: TelemetryQuantity;
  windowMs: number;
  hidden: number[];
  frame: { left: number; top: number; width: number; height: number };
  /** The direct labels' line height at the painter's font; OLP's canvas uses 11 px. */
  labelHeight?: number;
}

// A capture stands still until the next capture or Live, but the page asks
// again after every status poll: one plot of a 100 s capture (100 000 records,
// 6 axes) took 3.9 s in Qt 5.15's engine (pendant offscreen harness).
let capturedPlot: { key: string; capture: TelemetryRecorder; capabilities: unknown; plot: ReturnType<typeof plotOf> } | null = null;

/** Everything the plot shows, in pixels of `frame` and OLP's words. */
export function telemetryPlot(input: PlotInput) {
  if (!capture) return plotOf(input);
  const key = JSON.stringify([input, heldAtMs]);
  const capabilities = machineCapabilities();
  if (capturedPlot && capturedPlot.capture === capture && capturedPlot.capabilities === capabilities && capturedPlot.key === key) {
    return capturedPlot.plot;
  }
  const plot = plotOf(input);
  capturedPlot = { key, capture, capabilities, plot };
  return plot;
}

function plotOf(input: PlotInput) {
  const palette = input.dark ? TELEMETRY_PALETTE_DARK : TELEMETRY_PALETTE;
  const spec = quantitySpec(input.quantity);
  const shown = shownTelemetry(capture, live, input.windowMs, heldAtMs);
  const axes = machineCapabilities()?.rtCore?.axes ?? [];
  const count = Math.max(axes.length, shown.samples[0]?.axes.length ?? 0);
  const labelOf = (axis: number) => axes[axis]?.id ?? `J${axis + 1}`;
  const visible = Array.from({ length: count }, (_, axis) => axis).filter(axis => !input.hidden.includes(axis));
  const scaled = visible.map(axis => seriesFor(shown.samples, axis, spec.quantity, shown.smooth).map(v => v === null ? null : v * spec.scale));
  const range = plotRange(scaled);
  const tEnd = shown.samples.at(-1)?.tMs ?? 0;
  const scales = plotScales(input.frame, tEnd, shown.windowMs, range);
  const bottom = input.frame.top + input.frame.height;
  // At most two points per pixel column: a 1 kHz capture is thousands of
  // samples on a plot a few hundred pixels wide.
  const stride = Math.max(1, Math.ceil(shown.samples.length / Math.max(1, input.frame.width * 2)));
  const series = visible.map((axis, i) => {
    const runs: number[][] = [];
    let run: number[] = [];
    scaled[i]!.forEach((value, index) => {
      if (value === null) { if (run.length) runs.push(run); run = []; return; }
      if (index % stride !== 0 && index !== scaled[i]!.length - 1) return;
      run.push(scales.xOf(shown.samples[index]!.tMs), scales.yOf(value));
    });
    if (run.length) runs.push(run);
    return { axis, label: labelOf(axis), color: palette[axis % palette.length]!, runs };
  });
  const ends = spreadLabels(series.flatMap((line, i) => {
    const last = [...scaled[i]!].reverse().find(v => v !== null);
    return last === undefined || last === null ? [] : [{ label: line.label, color: line.color, y: scales.yOf(last) }];
  }), bottom, input.labelHeight);
  const captured = capture !== null;
  return {
    title: plotTitle(spec, telemetrySource(captured, captured ? null : live.rateHz(input.windowMs)), false),
    meta: telemetryMeta({ hasSamples: shown.samples.length > 0, captureNote: note, paused: heldAtMs !== null && !captured, showPreview: false }),
    captured,
    held: heldAtMs !== null,
    series,
    ends,
    zero: range.min < 0 && range.max > 0 ? scales.yOf(0) : null,
    valueTicks: valueTicks(range).map(tick => ({ y: scales.yOf(tick.value), label: tick.label })),
    timeTicks: timeTicks(tEnd, shown.windowMs, captured).map(tick => ({ x: scales.xOf(tick.t), label: tick.label })),
    legend: Array.from({ length: count }, (_, axis) => {
      const readout = axisReadout(shown, axis, spec);
      return {
        axis, label: labelOf(axis), color: palette[axis % palette.length]!, hidden: input.hidden.includes(axis),
        current: formatQuantity(readout.current, spec), peak: formatQuantity(readout.peak, spec), unit: spec.unit,
      };
    }),
  };
}
