/**
 * The telemetry strip: the drives' motion over time, drawn beside the Jog
 * controls so an operator sees whether a move was smooth or jerky while it
 * happens, and reads the values the ring records (position, drive velocity,
 * following error, torque) instead of guessing from the 3D model.
 *
 * The samples come from the Machine panel's status poll (40 Hz while the
 * viewer follows the machine, 4 Hz otherwise), so this is the coarse live
 * view; the 1 kHz record is the daemon's ring (`rtctl telemetry`). Units are
 * converted once at sampling time from the axis description the cell serves
 * (`counts_per_unit`, `sign`): a value the description cannot place in a unit
 * is not plotted.
 *
 * The failure this exists to prevent is silent: a model that does not move,
 * a jog that stutters on a slow link, a following error growing under load
 * with nothing on screen to show it.
 */
import { currentTheme } from "../ui/theme.ts";
import { previewJointRadForAxis } from "./livePose.ts";
import type { SimulationSample } from "../simulation/contracts.ts";
import { markBusy } from "../ui/busy.ts";
import {
  TELEMETRY_CAPTURE_DEFAULT_SECONDS, TELEMETRY_CAPTURE_SECONDS, TELEMETRY_PALETTE, TELEMETRY_PALETTE_DARK, TELEMETRY_QUANTITIES,
  TELEMETRY_WINDOWS_MS, TelemetryRecorder, axisReadout, captureFailedText, captureReadingText, formatQuantity, parseTelemetryView,
  plotRange, plotScales, plotTitle, quantitySpec, seriesFor, shownTelemetry, spreadLabels, telemetryMeta, telemetrySource,
  timeTicks, valueTicks,
  type ShownTelemetry, type TelemetryQuantity, type TelemetryQuantitySpec, type TelemetrySample, type TelemetryView,
} from "./telemetrySeries.ts";

// The arithmetic and the words live in telemetrySeries.ts, which the Steam
// Deck pendant runs too; its names stay importable from here.
export {
  TELEMETRY_PALETTE, TELEMETRY_PALETTE_DARK, TELEMETRY_QUANTITIES, TELEMETRY_RETENTION_MS, TELEMETRY_WINDOWS_MS,
  TelemetryRecorder, formatQuantity, niceRange, sampleFromStatus, samplesFromWindow, seriesFor,
} from "./telemetrySeries.ts";
export type { TelemetryAxisSample, TelemetryQuantity, TelemetryQuantitySpec, TelemetrySample } from "./telemetrySeries.ts";

/** Axis `index`'s hue in the window's current theme. */
export function telemetryColor(index: number): string {
  const palette = currentTheme() === "dark" ? TELEMETRY_PALETTE_DARK : TELEMETRY_PALETTE;
  return palette[index % palette.length]!;
}

/** One preview pose, stamped with the wall-clock it arrived, mapped onto
 *  machine-axis rows. Separate from `TelemetryRecorder` so the machine
 *  history is never a mix of live and planned. */
export interface PreviewTracePoint {
  /** Wall-clock milliseconds when the preview sample was shown. */
  tMs: number;
  /** Preview joint angle per machine axis, radians; null when unmapped. */
  positions: (number | null)[];
}

/**
 * The previewed plan's joint angles over wall-clock time. `previewSample(null)`
 * drops the series; nothing here is a velocity.
 */
export class PreviewTrace {
  #points: PreviewTracePoint[] = [];

  previewSample(
    sample: { jointOrder: readonly string[]; jointsRad: readonly number[] } | null,
    axisIds: readonly string[],
    tMs: number,
  ): void {
    if (!sample) {
      this.#points = [];
      return;
    }
    const count = axisIds.length > 0 ? axisIds.length : sample.jointsRad.length;
    const positions: (number | null)[] = [];
    for (let index = 0; index < count; index++) {
      positions.push(previewJointRadForAxis(sample, axisIds[index], index));
    }
    this.#points.push({ tMs, positions });
  }

  get active(): boolean {
    return this.#points.length > 0;
  }

  get length(): number {
    return this.#points.length;
  }

  get axisCount(): number {
    return this.#points.at(-1)?.positions.length ?? 0;
  }

  get points(): readonly PreviewTracePoint[] {
    return this.#points;
  }

  /** Position in radians for one axis, oldest first; empty when stopped. */
  series(axis: number): (number | null)[] {
    return this.#points.map(point => point.positions[axis] ?? null);
  }
}

interface StripOptions {
  /** Axis labels in axis order (J1…); missing entries print as A<n>. */
  axisLabels?: () => readonly string[];
  /** Fetch the newest `seconds` of the ring as samples; absent hides Capture. */
  capture?: (seconds: number, signal: AbortSignal) => Promise<{ samples: TelemetrySample[]; note: string }>;
  /** Persisted view choices; failures are swallowed. */
  storage?: Pick<Storage, "getItem" | "setItem"> | null;
}

const STORAGE_KEY = "olp.telemetry.view";

type StripView = TelemetryView;

function readView(storage: StripOptions["storage"]): StripView {
  let raw: string | null | undefined = null;
  try {
    raw = storage?.getItem(STORAGE_KEY);
  } catch {
    // Storage blocked: the default view.
  }
  return parseTelemetryView(raw);
}

/**
 * The strip's DOM and canvas. `push` records and schedules one repaint per
 * animation frame; nothing is drawn while the host is not displayed.
 */
export class TelemetryStrip {
  readonly recorder = new TelemetryRecorder();
  readonly preview = new PreviewTrace();
  readonly #host: HTMLElement;
  readonly #options: StripOptions;
  #view: StripView;
  #paused = false;
  /** The sample time a held plot stops at; null follows the live window. */
  #heldAtMs: number | null = null;
  #frame: number | null = null;
  #hover: { x: number; y: number } | null = null;
  /** A ring capture on show instead of the live history, and its note. */
  #capture: { recorder: TelemetryRecorder; note: string } | null = null;
  #capturing: AbortController | null = null;
  #captureButton!: HTMLButtonElement;
  #captureSeconds!: HTMLSelectElement;
  #liveButton!: HTMLButtonElement;
  #canvas!: HTMLCanvasElement;
  #tabs!: HTMLElement;
  #windowSelect!: HTMLSelectElement;
  #pauseButton!: HTMLButtonElement;
  #legend!: HTMLElement;
  #values!: HTMLElement;
  #meta!: HTMLElement;
  #tooltip!: HTMLElement;
  #axisCount = 0;

  constructor(host: HTMLElement, options: StripOptions = {}) {
    this.#host = host;
    this.#options = options;
    this.#view = readView(options.storage ?? (typeof localStorage === "undefined" ? null : localStorage));
    this.#build();
  }

  get quantity(): TelemetryQuantity { return this.#view.quantity; }
  get windowMs(): number { return this.#view.windowMs; }
  get paused(): boolean { return this.#paused; }
  get capturing(): boolean { return this.#capturing !== null; }
  get showingCapture(): boolean { return this.#capture !== null; }

  push(sample: TelemetrySample): void {
    if (!this.recorder.push(sample)) return;
    if (sample.axes.length !== this.#axisCount) {
      this.#axisCount = sample.axes.length;
      this.#renderLegend();
    }
    this.#schedule();
  }

  /** Overlay the previewed plan's joint angles on the position plot. Null
   *  drops the series. Never written into the machine recorder. */
  previewSample(sample: SimulationSample | null): void {
    const was = this.preview.active;
    const tMs = typeof performance !== "undefined" ? performance.now() : Date.now();
    this.preview.previewSample(sample, this.#options.axisLabels?.() ?? [], tMs);
    const n = Math.max(this.#axisCount, this.preview.axisCount);
    if (n !== this.#axisCount) {
      this.#axisCount = n;
      this.#renderLegend();
    } else if (this.preview.active !== was) {
      this.#renderLegend();
    }
    this.#schedule();
  }

  /** Preview position in radians for one axis; empty when the preview is stopped. */
  previewSeries(axis: number): (number | null)[] {
    return this.preview.series(axis);
  }

  clear(): void {
    this.recorder.clear();
    this.#schedule();
  }

  setQuantity(quantity: TelemetryQuantity): void {
    this.#view.quantity = quantity;
    this.#persist();
    this.#renderTabs();
    this.#schedule();
  }

  setWindow(windowMs: number): void {
    this.#view.windowMs = windowMs;
    this.#persist();
    this.#schedule();
  }

  setPaused(paused: boolean): void {
    this.#paused = paused;
    // Hold ends the live window at the newest sample now; sampling continues
    // into the recorder, and Resume follows it again.
    this.#heldAtMs = paused ? this.recorder.latest()?.tMs ?? null : null;
    this.#pauseButton.textContent = paused ? "Resume" : "Hold";
    this.#pauseButton.setAttribute("aria-pressed", String(paused));
    this.#pauseButton.title = paused ? "Follow the live samples again" : "Freeze the chart; sampling continues";
    this.#schedule();
  }

  toggleAxis(axis: number): void {
    const hidden = new Set(this.#view.hidden);
    if (hidden.has(axis)) hidden.delete(axis); else hidden.add(axis);
    this.#view.hidden = [...hidden].sort((a, b) => a - b);
    this.#persist();
    this.#renderLegend();
    this.#schedule();
  }

  /** Repaint now (tests and resize). */
  render(): void {
    this.#frame = null;
    this.#draw();
    this.#renderValues();
  }

  /** Read the newest `seconds` of the ring through the capture callback and
   *  show it in place of the live history. One capture at a time; a second
   *  press while fetching cancels the first. */
  async capture(seconds: number): Promise<void> {
    if (!this.#options.capture) return;
    if (this.#capturing) { this.#capturing.abort(); this.#capturing = null; this.#renderCaptureControls(); return; }
    const controller = new AbortController();
    this.#capturing = controller;
    this.#renderCaptureControls();
    this.#meta.textContent = captureReadingText(seconds);
    this.#meta.hidden = false;
    markBusy(this.#meta, true);
    try {
      const result = await this.#options.capture(seconds, controller.signal);
      if (controller.signal.aborted) return;
      this.showCapture(result.samples, result.note);
    } catch (error) {
      if (!controller.signal.aborted) this.#meta.textContent = captureFailedText(error instanceof Error ? error.message : String(error));
    } finally {
      if (this.#capturing === controller) { this.#capturing = null; this.#renderCaptureControls(); }
    }
  }

  /** Show captured samples (a ring window) instead of the live history. */
  showCapture(samples: readonly TelemetrySample[], note: string): void {
    const recorder = new TelemetryRecorder(Number.POSITIVE_INFINITY);
    for (const sample of samples) recorder.push(sample);
    this.#capture = { recorder, note };
    if (samples.length && samples[0]!.axes.length !== this.#axisCount) {
      this.#axisCount = samples[0]!.axes.length;
      this.#renderLegend();
    }
    this.#renderCaptureControls();
    this.#schedule();
  }

  showLive(): void {
    this.#capture = null;
    this.#renderCaptureControls();
    this.#schedule();
  }

  #renderCaptureControls(): void {
    const fetching = this.#capturing !== null;
    this.#captureButton.textContent = fetching ? "Cancel" : "Capture 1 kHz";
    this.#captureButton.classList.toggle("primary", !fetching);
    this.#captureSeconds.disabled = fetching;
    this.#liveButton.hidden = this.#capture === null;
    this.#host.classList.toggle("is-capture", this.#capture !== null);
  }

  /** The samples on show: the capture when one is up, else the live window,
   *  ending at the held sample while Hold is on. */
  #shown(): ShownTelemetry {
    return shownTelemetry(this.#capture?.recorder ?? null, this.recorder, this.#view.windowMs, this.#heldAtMs);
  }

  #persist(): void {
    try {
      (this.#options.storage ?? (typeof localStorage === "undefined" ? null : localStorage))?.setItem(STORAGE_KEY, JSON.stringify(this.#view));
    } catch {
      // Storage blocked: the choice lasts the session.
    }
  }

  #schedule(): void {
    if (this.#frame !== null) return;
    if (typeof requestAnimationFrame !== "function") { this.render(); return; }
    this.#frame = requestAnimationFrame(() => this.render());
  }

  #build(): void {
    const host = this.#host;
    host.classList.add("telemetry-strip");
    host.innerHTML = `
      <div class="telemetry-toolbar">
        <div class="telemetry-tabs" role="tablist" aria-label="Quantity"></div>
        <div class="telemetry-controls">
          <select class="telemetry-window" aria-label="Time window">
            ${TELEMETRY_WINDOWS_MS.map(ms => `<option value="${ms}">${ms / 1000} s</option>`).join("")}
          </select>
          <button type="button" class="mini-button telemetry-hold" aria-pressed="false" title="Freeze the chart; sampling continues">Hold</button>
          <button type="button" class="mini-button telemetry-clear" title="Forget the recorded samples">Clear</button>
          <span class="telemetry-capture-group" hidden>
            <select class="telemetry-capture-seconds" aria-label="Capture length">
              ${TELEMETRY_CAPTURE_SECONDS.map(s => `<option value="${s}"${s === TELEMETRY_CAPTURE_DEFAULT_SECONDS ? " selected" : ""}>${s} s</option>`).join("")}
            </select>
            <button type="button" class="mini-button primary telemetry-capture" title="Read the newest seconds of the machine's 1 kHz ring">Capture 1 kHz</button>
            <button type="button" class="mini-button telemetry-live" hidden title="Back to the live samples">Live</button>
          </span>
        </div>
      </div>
      <p class="telemetry-meta" data-telemetry-meta aria-live="polite"></p>
      <div class="telemetry-plot">
        <canvas class="telemetry-canvas" role="img" aria-label="Drive telemetry over time"></canvas>
        <div class="telemetry-tooltip" hidden></div>
      </div>
      <div class="telemetry-legend" role="group" aria-label="Axes: click to hide or show"></div>
      <table class="telemetry-values visually-hidden" aria-label="Latest values"><tbody></tbody></table>
    `;
    this.#canvas = host.querySelector<HTMLCanvasElement>(".telemetry-canvas")!;
    this.#tabs = host.querySelector<HTMLElement>(".telemetry-tabs")!;
    this.#windowSelect = host.querySelector<HTMLSelectElement>(".telemetry-window")!;
    this.#pauseButton = host.querySelector<HTMLButtonElement>(".telemetry-hold")!;
    this.#legend = host.querySelector<HTMLElement>(".telemetry-legend")!;
    this.#values = host.querySelector<HTMLElement>(".telemetry-values tbody")!;
    this.#meta = host.querySelector<HTMLElement>("[data-telemetry-meta]")!;
    this.#tooltip = host.querySelector<HTMLElement>(".telemetry-tooltip")!;
    this.#windowSelect.value = String(this.#view.windowMs);
    this.#windowSelect.addEventListener("change", () => this.setWindow(Number(this.#windowSelect.value)));
    this.#pauseButton.addEventListener("click", () => this.setPaused(!this.#paused));
    host.querySelector<HTMLButtonElement>(".telemetry-clear")!.addEventListener("click", () => this.clear());
    this.#captureButton = host.querySelector<HTMLButtonElement>(".telemetry-capture")!;
    this.#captureSeconds = host.querySelector<HTMLSelectElement>(".telemetry-capture-seconds")!;
    this.#liveButton = host.querySelector<HTMLButtonElement>(".telemetry-live")!;
    host.querySelector<HTMLElement>(".telemetry-capture-group")!.hidden = !this.#options.capture;
    this.#captureButton.addEventListener("click", () => void this.capture(Number(this.#captureSeconds.value)));
    this.#liveButton.addEventListener("click", () => this.showLive());
    this.#canvas.addEventListener("pointermove", event => {
      const rect = this.#canvas.getBoundingClientRect();
      this.#hover = { x: event.clientX - rect.left, y: event.clientY - rect.top };
      this.#schedule();
    });
    this.#canvas.addEventListener("pointerleave", () => { this.#hover = null; this.#tooltip.hidden = true; this.#schedule(); });
    if (typeof ResizeObserver === "function") new ResizeObserver(() => this.#schedule()).observe(this.#canvas);
    this.#renderTabs();
    this.#renderLegend();
  }

  #renderTabs(): void {
    this.#tabs.innerHTML = "";
    for (const spec of TELEMETRY_QUANTITIES) {
      const tab = document.createElement("button");
      tab.type = "button";
      tab.className = "run-switch-tab telemetry-tab" + (spec.quantity === this.#view.quantity ? " is-active" : "");
      tab.setAttribute("role", "tab");
      tab.setAttribute("aria-selected", String(spec.quantity === this.#view.quantity));
      tab.dataset.quantity = spec.quantity;
      tab.textContent = spec.short;
      tab.title = `${spec.label} in ${spec.unit}`;
      tab.addEventListener("click", () => this.setQuantity(spec.quantity));
      this.#tabs.append(tab);
    }
  }

  #axisLabel(index: number): string {
    return this.#options.axisLabels?.()[index] ?? `A${index + 1}`;
  }

  #renderLegend(): void {
    this.#legend.innerHTML = "";
    for (let axis = 0; axis < this.#axisCount; axis++) {
      const chip = document.createElement("button");
      chip.type = "button";
      chip.className = "telemetry-axis" + (this.#view.hidden.includes(axis) ? " is-hidden" : "");
      chip.setAttribute("aria-pressed", String(!this.#view.hidden.includes(axis)));
      chip.dataset.axis = String(axis);
      chip.title = `Show or hide ${this.#axisLabel(axis)}`;
      const swatch = document.createElement("span");
      swatch.className = "telemetry-swatch";
      swatch.style.background = telemetryColor(axis);
      const label = document.createElement("span");
      label.className = "telemetry-axis-name";
      label.textContent = this.#axisLabel(axis);
      const value = document.createElement("span");
      value.className = "telemetry-axis-value";
      value.dataset.axisValue = String(axis);
      value.textContent = "—";
      const peak = document.createElement("span");
      peak.className = "telemetry-axis-peak";
      peak.dataset.axisPeak = String(axis);
      const preview = document.createElement("span");
      preview.className = "telemetry-axis-preview";
      preview.dataset.axisPreview = String(axis);
      chip.append(swatch, label, value, peak, preview);
      chip.addEventListener("click", () => this.toggleAxis(axis));
      this.#legend.append(chip);
    }
    if (this.preview.active) {
      const sources = document.createElement("div");
      sources.className = "telemetry-source-legend";
      sources.setAttribute("role", "group");
      sources.setAttribute("aria-label", "Series");
      for (const [name, dashed] of [["machine", false], ["preview", true]] as const) {
        const item = document.createElement("span");
        item.className = "telemetry-source";
        const swatch = document.createElement("span");
        swatch.className = "telemetry-swatch" + (dashed ? " is-dashed" : "");
        const label = document.createElement("span");
        label.textContent = name;
        item.append(swatch, label);
        sources.append(item);
      }
      this.#legend.append(sources);
    }
  }

  #spec(): TelemetryQuantitySpec {
    return quantitySpec(this.#view.quantity);
  }

  #renderValues(): void {
    const spec = this.#spec();
    const shown = this.#shown();
    const samples = shown.samples;
    const latest = samples.at(-1);
    const showPreview = spec.quantity === "position" && this.preview.active && !this.#capture;
    this.#values.innerHTML = "";
    if (!this.#capturing) {
      this.#meta.textContent = telemetryMeta({ hasSamples: latest !== undefined, captureNote: this.#capture?.note ?? null, paused: this.#paused, showPreview });
      this.#meta.hidden = this.#meta.textContent === "";
      markBusy(this.#meta, false);
    }
    const head = document.createElement("tr");
    head.innerHTML = `<th scope="row">${spec.label} (${spec.unit})</th>`;
    const now = document.createElement("tr");
    now.innerHTML = `<th scope="row">now</th>`;
    const peak = document.createElement("tr");
    peak.innerHTML = `<th scope="row">largest in window</th>`;
    const previewRow = document.createElement("tr");
    previewRow.innerHTML = `<th scope="row">preview (${spec.unit})</th>`;
    for (let axis = 0; axis < this.#axisCount; axis++) {
      const valueCell = this.#legend.querySelector<HTMLElement>(`[data-axis-value="${axis}"]`);
      const peakCell = this.#legend.querySelector<HTMLElement>(`[data-axis-peak="${axis}"]`);
      const previewCell = this.#legend.querySelector<HTMLElement>(`[data-axis-preview="${axis}"]`);
      if (previewCell) previewCell.textContent = "";
      if (this.#view.hidden.includes(axis)) {
        if (valueCell) valueCell.textContent = "—";
        if (peakCell) peakCell.textContent = "";
        continue;
      }
      const { current, peak: max } = axisReadout(shown, axis, spec);
      head.insertAdjacentHTML("beforeend", `<th>${this.#axisLabel(axis)}</th>`);
      now.insertAdjacentHTML("beforeend", `<td>${formatQuantity(current, spec)}</td>`);
      peak.insertAdjacentHTML("beforeend", `<td>${formatQuantity(max, spec)}</td>`);
      if (valueCell) valueCell.textContent = formatQuantity(current, spec);
      if (peakCell) peakCell.textContent = max === null ? "" : `peak ${formatQuantity(max, spec)}`;
      if (showPreview) {
        const angle = this.preview.points.at(-1)?.positions[axis] ?? null;
        const text = formatQuantity(angle, spec);
        if (previewCell) previewCell.textContent = `preview ${text}${angle === null ? "" : spec.unit}`;
        previewRow.insertAdjacentHTML("beforeend", `<td>${text}</td>`);
      }
    }
    this.#values.append(head, now, peak);
    if (showPreview) this.#values.append(previewRow);
  }

  #draw(): void {
    const canvas = this.#canvas;
    if (!canvas.isConnected || canvas.offsetParent === null) return;
    const rect = canvas.getBoundingClientRect();
    if (rect.width < 10 || rect.height < 10) return;
    const dpr = window.devicePixelRatio || 1;
    const width = Math.round(rect.width * dpr), height = Math.round(rect.height * dpr);
    if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, rect.width, rect.height);
    const style = getComputedStyle(this.#host);
    const ink = style.getPropertyValue("--fg-dim").trim() || "#4a5259";
    const faint = style.getPropertyValue("--fg-faint").trim() || "#8f979e";
    const line = style.getPropertyValue("--line").trim() || "#b4b9be";
    const font = style.getPropertyValue("--t-caption").trim() || "500 11px sans-serif";

    const spec = this.#spec();
    const shown = this.#shown();
    const samples = shown.samples;
    const windowMs = shown.windowMs;
    const left = 44, right = 44, top = 28, bottom = 20;
    const plotW = rect.width - left - right, plotH = rect.height - top - bottom;
    if (plotW < 20 || plotH < 20) return;

    // Range over the visible axes, in display units. The preview's position
    // series is included so a moving plan is not clipped against a still arm.
    const drawPreview = spec.quantity === "position" && this.preview.active && !this.#capture;
    const nowMs = typeof performance !== "undefined" ? performance.now() : Date.now();
    const series: (number | null)[][] = [];
    const ranged: (number | null)[][] = [];
    for (let axis = 0; axis < this.#axisCount; axis++) {
      const values = this.#view.hidden.includes(axis) ? [] : seriesFor(samples, axis, spec.quantity, shown.smooth).map(v => v === null ? null : v * spec.scale);
      series.push(values);
      ranged.push(values);
      if (drawPreview && !this.#view.hidden.includes(axis)) ranged.push(this.preview.series(axis).map(v => v === null ? null : v * spec.scale));
    }
    const range = plotRange(ranged);
    const tEnd = samples.at(-1)?.tMs ?? 0;
    const { tStart, xOf, yOf } = plotScales({ left, top, width: plotW, height: plotH }, tEnd, windowMs, range);

    // Recessive grid and axis text.
    ctx.font = font;
    ctx.fillStyle = faint;
    ctx.strokeStyle = line;
    ctx.lineWidth = 1;
    ctx.globalAlpha = 0.6;
    for (const tick of valueTicks(range)) {
      const y = Math.round(yOf(tick.value)) + 0.5;
      ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(left + plotW, y); ctx.stroke();
      ctx.textAlign = "right"; ctx.textBaseline = "middle";
      ctx.fillText(tick.label, left - 6, y);
    }
    ctx.globalAlpha = 1;
    if (range.min < 0 && range.max > 0) {
      ctx.strokeStyle = faint;
      const y = Math.round(yOf(0)) + 0.5;
      ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(left + plotW, y); ctx.stroke();
    }
    ctx.textAlign = "center"; ctx.textBaseline = "top"; ctx.fillStyle = faint;
    for (const tick of timeTicks(tEnd, windowMs, this.#capture !== null)) ctx.fillText(tick.label, xOf(tick.t), top + plotH + 4);
    ctx.textAlign = "left"; ctx.textBaseline = "top"; ctx.fillStyle = ink;
    const rate = this.#capture ? null : this.recorder.rateHz(this.#view.windowMs);
    ctx.fillText(plotTitle(spec, telemetrySource(this.#capture !== null, rate), drawPreview), left, 6);

    // The lines: 1.5 px, one hue per axis, a direct label at the line's end.
    // Labels that would land on each other are pushed apart by a line height.
    ctx.lineWidth = 1.5;
    ctx.lineJoin = "round";
    const ends: { axis: number; y: number }[] = [];
    for (let axis = 0; axis < this.#axisCount; axis++) {
      const values = series[axis]!;
      if (!values.length) continue;
      ctx.strokeStyle = telemetryColor(axis);
      ctx.beginPath();
      let pen = false;
      let lastY = 0;
      for (let i = 0; i < samples.length; i++) {
        const v = values[i];
        if (v == null) { pen = false; continue; }
        const x = xOf(samples[i]!.tMs), y = yOf(v);
        if (pen) ctx.lineTo(x, y); else ctx.moveTo(x, y);
        pen = true; lastY = y;
      }
      ctx.stroke();
      if (pen) ends.push({ axis, y: lastY });
    }
    if (drawPreview) {
      ctx.setLineDash([5, 4]);
      for (let axis = 0; axis < this.#axisCount; axis++) {
        if (this.#view.hidden.includes(axis)) continue;
        ctx.strokeStyle = telemetryColor(axis);
        ctx.beginPath();
        let pen = false;
        for (const point of this.preview.points) {
          const v = point.positions[axis];
          if (v == null) { pen = false; continue; }
          const x = xOf(tEnd - (nowMs - point.tMs)), y = yOf(v * spec.scale);
          if (pen) ctx.lineTo(x, y); else ctx.moveTo(x, y);
          pen = true;
        }
        ctx.stroke();
      }
      ctx.setLineDash([]);
    }
    spreadLabels(ends, top + plotH, 11);
    ctx.textAlign = "left"; ctx.textBaseline = "middle";
    for (const end of ends) {
      ctx.fillStyle = telemetryColor(end.axis);
      ctx.fillRect(left + plotW + 3, Math.round(end.y) - 1, 6, 2);
      ctx.fillStyle = ink;
      ctx.fillText(this.#axisLabel(end.axis), left + plotW + 12, Math.min(Math.max(end.y, top + 5), top + plotH - 5));
    }

    // Hover: a crosshair at the nearest sample and the tooltip beside it.
    if (this.#hover && (samples.length || drawPreview) && this.#hover.x >= left && this.#hover.x <= left + plotW) {
      const t = tStart + ((this.#hover.x - left) / plotW) * windowMs;
      let nearest = 0;
      for (let i = 1; i < samples.length; i++) if (Math.abs(samples[i]!.tMs - t) < Math.abs(samples[nearest]!.tMs - t)) nearest = i;
      const hoverTime = drawPreview ? t : samples[nearest]!.tMs;
      const x = Math.round(xOf(hoverTime)) + 0.5;
      ctx.strokeStyle = ink; ctx.globalAlpha = 0.5;
      ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x, top + plotH); ctx.stroke();
      ctx.globalAlpha = 1;
      const rows: string[] = [`<div class="telemetry-tooltip-time">−${((tEnd - hoverTime) / 1000).toFixed(2)} s</div>`];
      for (let axis = 0; axis < this.#axisCount; axis++) {
        const v = series[axis]?.[nearest];
        if (v === undefined || v === null) continue;
        rows.push(`<div><span class="telemetry-swatch" style="background:${telemetryColor(axis)}"></span>${this.#axisLabel(axis)}${drawPreview ? " machine" : ""} <b>${v.toFixed(spec.decimals)}</b> ${spec.unit}</div>`);
      }
      if (drawPreview) {
        const previewTime = nowMs - (tEnd - hoverTime);
        const points = this.preview.points;
        // Do not extrapolate preview angles into time before playback began.
        if (previewTime >= points[0]!.tMs && previewTime <= points.at(-1)!.tMs) {
          let point = points[0]!;
          for (const candidate of points) if (Math.abs(candidate.tMs - previewTime) < Math.abs(point.tMs - previewTime)) point = candidate;
          for (let axis = 0; axis < this.#axisCount; axis++) {
            const angle = point.positions[axis];
            if (this.#view.hidden.includes(axis) || angle == null) continue;
            rows.push(`<div><span class="telemetry-swatch is-dashed" style="background:${telemetryColor(axis)}"></span>${this.#axisLabel(axis)} preview <b>${formatQuantity(angle, spec)}</b> ${spec.unit}</div>`);
          }
        }
      }
      this.#tooltip.innerHTML = rows.join("");
      this.#tooltip.hidden = false;
      const flip = this.#hover.x > rect.width * 0.6;
      this.#tooltip.style.left = flip ? "" : `${x + 10}px`;
      this.#tooltip.style.right = flip ? `${rect.width - x + 10}px` : "";
      this.#tooltip.style.top = `${Math.max(0, Math.min(this.#hover.y - 10, rect.height - 40))}px`;
    } else {
      this.#tooltip.hidden = true;
    }
  }
}

