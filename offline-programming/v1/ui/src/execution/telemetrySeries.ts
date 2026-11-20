/**
 * The telemetry strip's arithmetic without its DOM: what a sample is, how the
 * drives' raw units become display units, which samples are on show, the
 * plotted range and ticks, and every word the strip prints about them.
 * `telemetryStrip.ts` draws this on a canvas; the Steam Deck pendant runs this
 * same module inside Qt's engine and paints it with QPainter, so an operator
 * reads the same number, in the same unit, under the same caption on both.
 *
 * The failure this exists to prevent is two plots of one drive that disagree:
 * a second conversion that forgets the axis sign, plots an invalid sample as
 * zero, or smooths a capture differently.
 */
import type { DenseCapabilities, DenseStatus, DenseTelemetryWindow } from "./denseClient.ts";

export type TelemetryQuantity = "position" | "velocity" | "acceleration" | "following_error" | "torque";

/** One axis at one poll, in the axis unit (rad for a rotary axis) except
 *  torque, which the drive reports in per-mille of rated torque. */
export interface TelemetryAxisSample {
  position: number | null;
  target: number | null;
  /** The drive's own velocity measurement, unit/s. */
  velocity: number | null;
  /** Commanded minus actual, in the axis unit. */
  followingError: number | null;
  /** Per-mille of rated torque, signed. */
  torque: number | null;
}

export interface TelemetrySample {
  /** Daemon host time in milliseconds (monotonic on the cell). */
  tMs: number;
  axes: TelemetryAxisSample[];
}

export interface TelemetryQuantitySpec {
  quantity: TelemetryQuantity;
  label: string;
  /** The tab's text; the plot title carries the full label. */
  short: string;
  /** Unit label after conversion for display. */
  unit: string;
  /** Multiplies the stored SI value into the display unit. */
  scale: number;
  decimals: number;
}

const RAD_TO_DEG = 180 / Math.PI;

/** Display units: degrees for angles, millidegrees for following error (a
 *  few hundred counts on a 2 million count/rad axis is hundredths of a degree
 *  and would print as 0.00), per-mille for torque. */
export const TELEMETRY_QUANTITIES: readonly TelemetryQuantitySpec[] = [
  { quantity: "position", label: "Position", short: "Position", unit: "°", scale: RAD_TO_DEG, decimals: 2 },
  { quantity: "velocity", label: "Velocity", short: "Velocity", unit: "°/s", scale: RAD_TO_DEG, decimals: 2 },
  { quantity: "acceleration", label: "Acceleration", short: "Accel.", unit: "°/s²", scale: RAD_TO_DEG, decimals: 1 },
  { quantity: "following_error", label: "Following error", short: "Follow. error", unit: "m°", scale: RAD_TO_DEG * 1000, decimals: 1 },
  { quantity: "torque", label: "Torque", short: "Torque", unit: "‰", scale: 1, decimals: 0 },
];

export const TELEMETRY_WINDOWS_MS: readonly number[] = [10_000, 30_000, 60_000];

/** Capture lengths offered, in seconds of the 1 kHz ring. The server's
 *  handler (`dense_execution.go`) refuses more than 30 s, so the 100 s choice
 *  answers with that refusal rather than a capture. */
export const TELEMETRY_CAPTURE_SECONDS: readonly number[] = [1, 3, 10, 30, 100];
export const TELEMETRY_CAPTURE_DEFAULT_SECONDS = 3;

/** Fixed hue per axis index, never cycled: J1 is always the first colour.
 *  Validated with the dataviz palette checks on the panel surface #dcdfe2
 *  (adjacent-pair CVD ΔE ≥ 9.1, normal-vision ΔE ≥ 19.6); the contrast
 *  warning is met by the direct labels at each line's end and the values
 *  table under the chart. */
export const TELEMETRY_PALETTE: readonly string[] = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#4a3aa7"];
/** The same six hues with their lightness moved into the dark band, for the
 *  dark theme's plot surface #1f2226: validated 2026-09-26 with the dataviz
 *  palette checks (OKLCH L 0.55-0.67, adjacent-pair CVD ΔE ≥ 9.0,
 *  normal-vision ΔE ≥ 18.5, every hue ≥ 3:1 on the surface). The light
 *  palette's indigo fell to 1.87:1 there. */
export const TELEMETRY_PALETTE_DARK: readonly string[] = ["#2c79d8", "#e05e29", "#14ac77", "#c78600", "#c75d87", "#685ece"];

/** Longest history kept, so a wider window chosen later has data. */
export const TELEMETRY_RETENTION_MS = 120_000;

/** The Machine panel's status as one sample. Null when the status carries no
 *  rt-core axes, or when the description cannot say what a count is. */
export function sampleFromStatus(status: DenseStatus | null, capabilities: DenseCapabilities | null): TelemetrySample | null {
  if (!status || status.backend !== "rt_core" || !status.rtCore) return null;
  const rt = status.rtCore;
  const described = capabilities?.backend === "rt_core" ? capabilities.rtCore?.axes ?? [] : [];
  if (!rt.axes.length || !described.length) return null;
  const tMs = rt.timeNs / 1e6;
  if (!Number.isFinite(tMs) || tMs <= 0) return null;
  const axes = rt.axes.map((axis, index) => {
    const description = described[index];
    const perUnit = description?.counts_per_unit;
    const sign = description?.sign === -1 ? -1 : 1;
    // `|| 0` folds the -0 a zero count times a negative sign would print as.
    const toUnit = perUnit && Number.isFinite(perUnit) && perUnit > 0
      ? (counts: number) => (counts * sign) / perUnit || 0
      : null;
    return {
      position: axis.position,
      target: axis.target,
      velocity: toUnit && axis.velocityActualValid && Number.isFinite(axis.velocityActualCountsPerS) ? toUnit(axis.velocityActualCountsPerS) : null,
      followingError: toUnit && axis.followingErrorValid && Number.isFinite(axis.followingErrorCounts) ? toUnit(axis.followingErrorCounts) : null,
      torque: Number.isFinite(axis.torqueRaw) ? axis.torqueRaw : null,
    };
  });
  return { tMs, axes };
}

/** A ring window as samples, in axis units. The ring holds encoder counts,
 *  not the logical position the live status reports, so positions are taken
 *  relative to the window's first record: the shape of the motion is the
 *  point, and the absolute angle is on the live view. A record whose axis
 *  measurement the drive marked invalid is null there, as on the live view. */
export function samplesFromWindow(window: DenseTelemetryWindow, capabilities: DenseCapabilities | null): TelemetrySample[] {
  const described = capabilities?.backend === "rt_core" ? capabilities.rtCore?.axes ?? [] : [];
  if (!window.records.length || !described.length) return [];
  const first = window.records[0]!;
  return window.records.map(record => ({
    tMs: record.t_ns / 1e6,
    axes: record.ax.map((axis, index) => {
      const description = described[index];
      const perUnit = description?.counts_per_unit;
      const sign = description?.sign === -1 ? -1 : 1;
      const toUnit = perUnit && Number.isFinite(perUnit) && perUnit > 0 ? (counts: number) => (counts * sign) / perUnit || 0 : null;
      const origin = first.ax[index]?.p ?? 0;
      return {
        position: toUnit ? toUnit(axis.p - origin) : null,
        target: toUnit ? toUnit(axis.tp - origin) : null,
        velocity: toUnit && axis.v_ok ? toUnit(axis.v) : null,
        followingError: toUnit && axis.fe_ok ? toUnit(axis.fe) : null,
        torque: axis.tq_ok ? axis.tq : null,
      };
    }),
  }));
}

/** Time-bounded history of samples, oldest first. A sample with a time at or
 *  before the last kept one is dropped: the poll can deliver the same status
 *  twice, and a daemon restart moves time backwards. */
export class TelemetryRecorder {
  #samples: TelemetrySample[] = [];
  readonly retentionMs: number;

  constructor(retentionMs = TELEMETRY_RETENTION_MS) {
    this.retentionMs = retentionMs;
  }

  push(sample: TelemetrySample): boolean {
    const last = this.#samples.at(-1);
    if (last && sample.tMs <= last.tMs) {
      // Time went backwards by more than the retention: the daemon restarted;
      // start over rather than hold a history the new clock cannot extend.
      if (sample.tMs < last.tMs - this.retentionMs) this.#samples = [];
      else return false;
    }
    this.#samples.push(sample);
    const floor = sample.tMs - this.retentionMs;
    let drop = 0;
    while (drop < this.#samples.length && this.#samples[drop]!.tMs < floor) drop++;
    if (drop) this.#samples.splice(0, drop);
    return true;
  }

  clear(): void {
    this.#samples = [];
  }

  get length(): number {
    return this.#samples.length;
  }

  latest(): TelemetrySample | null {
    return this.#samples.at(-1) ?? null;
  }

  /** Samples inside the last `windowMs`, oldest first. */
  window(windowMs: number): TelemetrySample[] {
    const last = this.#samples.at(-1);
    if (!last) return [];
    const floor = last.tMs - windowMs;
    let start = this.#samples.length - 1;
    while (start > 0 && this.#samples[start - 1]!.tMs >= floor) start--;
    return this.#samples.slice(start);
  }

  /** Samples per second over the window, from the timestamps themselves. */
  rateHz(windowMs: number): number | null {
    const samples = this.window(windowMs);
    if (samples.length < 2) return null;
    const span = samples.at(-1)!.tMs - samples[0]!.tMs;
    return span > 0 ? ((samples.length - 1) * 1000) / span : null;
  }
}

/** One axis's series for a quantity over a window of samples: pairs of
 *  [tMs, value] with nulls where the quantity was not observed. Acceleration
 *  is the central difference of the drive's velocity, so it needs three
 *  samples and is null at the ends. */
export function seriesFor(samples: readonly TelemetrySample[], axis: number, quantity: TelemetryQuantity, smooth = 1): (number | null)[] {
  if (quantity === "acceleration") {
    // At 1 kHz the drive's velocity is quantised to a count per cycle, so the
    // raw difference is noise; a moving average of `smooth` samples on each
    // side of the difference recovers the profile. Live samples use 1.
    const half = Math.max(0, Math.floor(smooth / 2));
    const velocity = (index: number): number | null => {
      let sum = 0, n = 0;
      for (let i = index - half; i <= index + half; i++) {
        const v = samples[i]?.axes[axis]?.velocity;
        if (v == null) continue;
        sum += v; n++;
      }
      return n === 0 ? null : sum / n;
    };
    return samples.map((sample, index) => {
      const before = samples[index - 1], after = samples[index + 1];
      const v0 = before === undefined ? null : velocity(index - 1), v1 = after === undefined ? null : velocity(index + 1);
      if (before === undefined || after === undefined || v0 == null || v1 == null) return null;
      const dt = (after.tMs - before.tMs) / 1000;
      return dt > 0 ? (v1 - v0) / dt : null;
    });
  }
  return samples.map(sample => {
    const a = sample.axes[axis];
    if (!a) return null;
    switch (quantity) {
      case "position": return a.position;
      case "velocity": return a.velocity;
      case "following_error": return a.followingError;
      case "torque": return a.torque;
    }
  });
}

/** Round the plotted range out to a tick step a reader can count in (1, 2, 5
 *  times a power of ten), with room for the direct labels. */
export function niceRange(min: number, max: number, ticks = 5): { min: number; max: number; step: number } {
  if (!Number.isFinite(min) || !Number.isFinite(max)) return { min: -1, max: 1, step: 0.5 };
  if (min === max) { min -= 1; max += 1; }
  const span = max - min;
  const raw = span / ticks;
  const magnitude = 10 ** Math.floor(Math.log10(raw));
  const residual = raw / magnitude;
  const step = (residual >= 5 ? 10 : residual >= 2 ? 5 : residual >= 1 ? 2 : 1) * magnitude;
  return { min: Math.floor(min / step) * step, max: Math.ceil(max / step) * step, step };
}

export function formatQuantity(value: number | null, spec: TelemetryQuantitySpec): string {
  if (value === null || !Number.isFinite(value)) return "—";
  return (value * spec.scale).toFixed(spec.decimals);
}

export function formatTick(value: number, step: number): string {
  const decimals = step >= 1 ? 0 : Math.min(3, Math.ceil(-Math.log10(step)));
  const text = value.toFixed(decimals);
  return text === "-0" || /^-0\.0+$/.test(text) ? text.slice(1) : text;
}

/** The operator's choices: the tab, the time window and the hidden axes. */
export interface TelemetryView {
  quantity: TelemetryQuantity;
  windowMs: number;
  hidden: number[];
}

/** A stored view, or the default (Velocity over 10 s, every axis) for
 *  anything a stored value cannot name: an older build's quantity, a window
 *  no longer offered, a hidden list that is not integers. */
export function parseTelemetryView(raw: string | null | undefined): TelemetryView {
  const fallback: TelemetryView = { quantity: "velocity", windowMs: 10_000, hidden: [] };
  try {
    if (!raw) return fallback;
    const parsed = JSON.parse(raw) as Partial<TelemetryView>;
    return {
      quantity: TELEMETRY_QUANTITIES.some(q => q.quantity === parsed.quantity) ? parsed.quantity as TelemetryQuantity : fallback.quantity,
      windowMs: TELEMETRY_WINDOWS_MS.includes(parsed.windowMs as number) ? parsed.windowMs as number : fallback.windowMs,
      hidden: Array.isArray(parsed.hidden) ? parsed.hidden.filter((n): n is number => Number.isInteger(n)) : [],
    };
  } catch {
    return fallback;
  }
}

export function quantitySpec(quantity: TelemetryQuantity): TelemetryQuantitySpec {
  return TELEMETRY_QUANTITIES.find(q => q.quantity === quantity) ?? TELEMETRY_QUANTITIES[0]!;
}

/** The samples a plot shows and how acceleration is smoothed over them. */
export interface ShownTelemetry {
  samples: TelemetrySample[];
  windowMs: number;
  smooth: number;
}

/** The capture when one is up, spanning exactly its records; otherwise the
 *  live window. `heldAtMs` ends a live window at that sample time instead of
 *  the newest, so a held plot stops moving while sampling continues. */
export function shownTelemetry(capture: TelemetryRecorder | null, live: TelemetryRecorder, windowMs: number, heldAtMs: number | null = null): ShownTelemetry {
  if (capture) {
    const samples = capture.window(Number.POSITIVE_INFINITY);
    const span = samples.length > 1 ? samples.at(-1)!.tMs - samples[0]!.tMs : 1000;
    const rate = samples.length > 1 ? ((samples.length - 1) * 1000) / span : 0;
    // Nine samples of smoothing at 1 kHz is 9 ms; dense captures only.
    return { samples, windowMs: Math.max(span, 1), smooth: rate > 200 ? 9 : 1 };
  }
  if (heldAtMs !== null) {
    const samples = live.window(Number.POSITIVE_INFINITY).filter(sample => sample.tMs <= heldAtMs && sample.tMs >= heldAtMs - windowMs);
    return { samples, windowMs, smooth: 1 };
  }
  return { samples: live.window(windowMs), windowMs, smooth: 1 };
}

/** The legend's readout for one axis: the newest value and the one of
 *  largest magnitude in the shown samples, in stored units. */
export function axisReadout(shown: ShownTelemetry, axis: number, spec: TelemetryQuantitySpec): { current: number | null; peak: number | null } {
  const series = seriesFor(shown.samples, axis, spec.quantity, shown.smooth);
  const current = series.at(-1) ?? null;
  let peak: number | null = null;
  for (const value of series) if (value !== null && (peak === null || Math.abs(value) > Math.abs(peak))) peak = value;
  return { current, peak };
}

/** The line above the plot while no capture is being read. */
export function telemetryMeta(input: { hasSamples: boolean; captureNote: string | null; paused: boolean; showPreview: boolean }): string {
  if (input.captureNote !== null) {
    return input.hasSamples ? `${input.captureNote}. Positions are relative to the window start.` : `${input.captureNote}. No records.`;
  }
  return input.hasSamples
    ? (input.paused ? "Held. Sampling continues; press Resume to follow." : "")
    : input.showPreview ? "Preview angles shown; machine telemetry unavailable." : "Connect a machine to see its drives here.";
}

export function captureReadingText(seconds: number): string {
  return `reading the newest ${seconds} s of the ring…`;
}

export function captureFailedText(message: string): string {
  return `capture failed: ${message}`;
}

/** Where the plotted samples came from, for the plot's title. */
export function telemetrySource(captured: boolean, rateHz: number | null): string {
  return captured ? "1 kHz capture" : rateHz ? `live at ${rateHz.toFixed(0)} Hz` : "live";
}

export function plotTitle(spec: TelemetryQuantitySpec, source: string, preview: boolean): string {
  return `${spec.label} in ${spec.unit}, ${source}${preview ? " + preview" : ""}`;
}

/** The value range over every plotted series in display units, rounded out;
 *  -1..1 when nothing is plotted. */
export function plotRange(series: readonly (readonly (number | null)[])[]): { min: number; max: number; step: number } {
  let min = Infinity, max = -Infinity;
  for (const values of series) for (const v of values) if (v !== null) { if (v < min) min = v; if (v > max) max = v; }
  if (!Number.isFinite(min)) { min = -1; max = 1; }
  return niceRange(min, max);
}

/** The plot's pixel mapping: time runs left to right over `windowMs` ending
 *  at `tEnd`; the range fills the height, larger values up. */
export function plotScales(frame: { left: number; top: number; width: number; height: number }, tEnd: number, windowMs: number, range: { min: number; max: number }) {
  const tStart = tEnd - windowMs;
  return {
    tStart,
    xOf: (t: number) => frame.left + ((t - tStart) / windowMs) * frame.width,
    yOf: (v: number) => frame.top + (1 - (v - range.min) / (range.max - range.min)) * frame.height,
  };
}

/** One gridline per tick step from the range's floor to its ceiling. */
export function valueTicks(range: { min: number; max: number; step: number }): { value: number; label: string }[] {
  const ticks: { value: number; label: string }[] = [];
  for (let v = range.min; v <= range.max + range.step / 2; v += range.step) ticks.push({ value: v, label: formatTick(v, range.step) });
  return ticks;
}

/** Time labels under the plot: seconds before the newest sample, ending in
 *  "now" (live) or "end" (a capture). */
export function timeTicks(tEnd: number, windowMs: number, captured: boolean): { t: number; label: string }[] {
  const tStart = tEnd - windowMs;
  const seconds = windowMs / 1000;
  const count = seconds <= 10 ? 5 : 6;
  const ticks: { t: number; label: string }[] = [];
  for (let i = 0; i <= count; i++) {
    const t = tStart + (i / count) * windowMs;
    ticks.push({ t, label: i === count ? (captured ? "end" : "now") : `−${((tEnd - t) / 1000).toFixed(seconds < 5 ? 1 : 0)} s` });
  }
  return ticks;
}

/** Direct labels at the lines' ends, sorted top to bottom and pushed apart by
 *  a line height so they never print over each other; `bottom` is the plot's
 *  lower edge. Mutates and returns `ends`. */
export function spreadLabels<T extends { y: number }>(ends: T[], bottom: number, lineHeight = 11): T[] {
  ends.sort((a, b) => a.y - b.y);
  for (let i = 1; i < ends.length; i++) {
    if (ends[i]!.y - ends[i - 1]!.y < lineHeight) ends[i]!.y = ends[i - 1]!.y + lineHeight;
  }
  for (let i = ends.length - 2; i >= 0; i--) {
    if (ends[i + 1]!.y > bottom && ends[i + 1]!.y - ends[i]!.y < lineHeight) ends[i]!.y = ends[i + 1]!.y - lineHeight;
  }
  return ends;
}

/** The note a capture is shown under: how many records over what span, their
 *  sequence numbers and how long the server took to read them. */
export function captureNote(window: DenseTelemetryWindow): string {
  const span = window.records.length ? (window.records.at(-1)!.t_ns - window.records[0]!.t_ns) / 1e9 : 0;
  return `1 kHz capture · ${window.records.length} records over ${span.toFixed(2)} s · sequences ${window.first_sequence}–${window.last_sequence} · fetched in ${(window.fetch_ms / 1000).toFixed(1)} s`;
}
