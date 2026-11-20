/**
 * Simulate: replay a planned rails program in the viewer, at its own timing.
 *
 * Separate from Plan, and separate from running it. Planning produces the
 * trajectory; simulating watches it; loading it into the daemon moves an arm.
 * They were one button when planning also compiled rows and the canonical
 * simulator re-planned from them -- and that button hid the fact that a rails
 * program was being planned twice, the weld planner's certified motion
 * flattened to poses and handed to Tesseract to plan again.
 *
 * What is replayed here is the blob, so what is watched is what will run.
 *
 * The clock is the trajectory's. Samples arrive at whatever rate the planner
 * emitted -- 100 Hz on the corpus -- and this walks them against wall time
 * rather than one per frame, so a slow frame drops samples instead of slowing
 * the weld down. Nothing here commands anything: it poses a viewer.
 */

import type { SimulationSample } from "../simulation/contracts.ts";

export type SimulationPhase = "idle" | "playing" | "paused" | "ended";

export interface DenseSimulationSnapshot {
  phase: SimulationPhase;
  cursorS: number;
  durationS: number;
  sampleIndex: number;
  torchOn: boolean;
  /** Trajectory seconds per wall second: 1 is the weld's own pace. */
  rate: number;
}

/** The playback rates the transport offers. Powers of two, so 16x of a
 *  100 Hz trajectory is still one sample per 0.6 ms frame budget and the
 *  scrubber's arithmetic stays exact. */
export const PLAYBACK_RATES = [1, 2, 4, 8, 16] as const;
export type PlaybackRate = (typeof PLAYBACK_RATES)[number];

export interface DenseSimulationDeps {
  /** Wall clock, in milliseconds. Injected so a test does not wait. */
  now(): number;
  /** Schedule the next step. Returns a handle `cancel` understands. */
  schedule(step: () => void): number;
  cancel(handle: number): void;
  /** Show one sample. */
  show(sample: SimulationSample | null): void;
  /** Phase, cursor and torch changed. */
  onChange(snapshot: DenseSimulationSnapshot): void;
}

/**
 * A player over one decoded trajectory.
 *
 * Replays the same samples forward or backward, optionally stopping at a
 * planned move boundary.
 */
export class DenseSimulation {
  #samples: SimulationSample[] = [];
  #torch: boolean[] = [];
  #phase: SimulationPhase = "idle";
  #cursorS = 0;
  #startedAtMs = 0;
  #startedFromS = 0;
  #rate: PlaybackRate = 1;
  #handle: number | null = null;
  #stepTargetS: number | null = null;
  #direction = 1;
  readonly #deps: DenseSimulationDeps;

  constructor(deps: DenseSimulationDeps) {
    this.#deps = deps;
  }

  /** Load a trajectory and show its first sample. Replaces whatever was
   *  loaded, because two trajectories on one viewer is two answers. */
  load(samples: SimulationSample[], torchOn: boolean[]): void {
    this.stop();
    this.#samples = samples;
    this.#torch = torchOn;
    this.#cursorS = 0;
    this.#phase = samples.length > 0 ? "paused" : "idle";
    this.#deps.show(samples[0] ?? null);
    this.#emit();
  }

  get durationS(): number {
    const last = this.#samples[this.#samples.length - 1];
    return last ? last.timeS : 0;
  }

  get phase(): SimulationPhase {
    return this.#phase;
  }

  get rate(): PlaybackRate {
    return this.#rate;
  }

  /** Change the pace, playing or not. Mid-play the clock is re-based at the
   *  cursor so the change applies from now rather than re-timing what has
   *  already been shown; the cursor itself does not move. */
  setRate(rate: PlaybackRate): void {
    if (rate === this.#rate) return;
    if (this.#phase === "playing") {
      this.#cursorS = this.#cursorAtNow();
      this.#startedAtMs = this.#deps.now();
      this.#startedFromS = this.#cursorS;
    }
    this.#rate = rate;
    this.#emit();
  }

  play(): void {
    if (this.#samples.length === 0 || this.#phase === "playing") return;
    // Replaying from the end starts over, which is what pressing play on a
    // finished thing means everywhere else.
    if (this.#cursorS >= this.durationS - 1e-9) this.#cursorS = 0;
    this.#stepTargetS = null;
    this.#direction = 1;
    this.#phase = "playing";
    this.#startedAtMs = this.#deps.now();
    this.#startedFromS = this.#cursorS;
    this.#emit();
    this.#tick();
  }

  canStep(direction: "forward" | "backward", boundaries: readonly number[]): boolean {
    return this.#samples.length > 0 && this.#stepTarget(direction, boundaries) !== undefined;
  }

  step(direction: "forward" | "backward", boundaries: readonly number[]): void {
    const target = this.#stepTarget(direction, boundaries);
    if (this.#samples.length === 0 || target === undefined) return;
    this.#clear();
    this.#direction = direction === "forward" ? 1 : -1;
    this.#stepTargetS = target;
    this.#phase = "playing";
    this.#startedAtMs = this.#deps.now();
    this.#startedFromS = this.#cursorS;
    this.#tick();
  }

  #stepTarget(direction: "forward" | "backward", boundaries: readonly number[]): number | undefined {
    return direction === "forward"
      ? boundaries.find(time => time > this.#cursorS + 1e-9)
      : [...boundaries].reverse().find(time => time < this.#cursorS - 1e-9);
  }

  pause(): void {
    if (this.#phase !== "playing") return;
    this.#phase = "paused";
    this.#clear();
    this.#emit();
  }

  /** Move to a time and show it, playing or not. */
  seek(seconds: number): void {
    if (this.#samples.length === 0) return;
    this.#stepTargetS = null;
    this.#direction = 1;
    this.#cursorS = Math.min(Math.max(seconds, 0), this.durationS);
    this.#startedAtMs = this.#deps.now();
    this.#startedFromS = this.#cursorS;
    this.#deps.show(this.#sampleAtCursor());
    this.#emit();
  }

  stop(): void {
    this.#clear();
    this.#stepTargetS = null;
    this.#direction = 1;
    this.#phase = this.#samples.length > 0 ? "paused" : "idle";
    this.#cursorS = 0;
    if (this.#samples.length > 0) this.#deps.show(this.#samples[0]!);
    this.#emit();
  }

  /** Where the clock says the cursor is, at the current rate. */
  #cursorAtNow(): number {
    const elapsed = (this.#deps.now() - this.#startedAtMs) / 1000;
    const time = this.#startedFromS + elapsed * this.#rate * this.#direction;
    const end = this.#stepTargetS ?? this.durationS;
    return this.#direction > 0 ? Math.min(time, end) : Math.max(time, end);
  }

  #tick(): void {
    if (this.#phase !== "playing") return;
    this.#cursorS = this.#cursorAtNow();
    if (this.#stepTargetS !== null && Math.abs(this.#cursorS - this.#stepTargetS) < 1e-9) {
      this.#phase = "paused";
      this.#clear();
      this.#deps.show(this.#sampleAtCursor());
      this.#emit();
      return;
    }
    if (this.#stepTargetS === null && this.#cursorS >= this.durationS) {
      this.#cursorS = this.durationS;
      this.#phase = "ended";
      this.#clear();
      this.#deps.show(this.#samples[this.#samples.length - 1] ?? null);
      this.#emit();
      return;
    }
    this.#deps.show(this.#sampleAtCursor());
    this.#emit();
    this.#handle = this.#deps.schedule(() => this.#tick());
  }

  /** The last sample at or before the cursor.
   *
   *  At or BEFORE, never interpolated between two. The samples are the
   *  planner's own and the daemon plays them one at a time; a viewer showing
   *  a pose between two of them would be showing a pose that never occurs. */
  #sampleAtCursor(): SimulationSample | null {
    if (this.#samples.length === 0) return null;
    let low = 0;
    let high = this.#samples.length - 1;
    while (low < high) {
      const middle = (low + high + 1) >> 1;
      if (this.#samples[middle]!.timeS <= this.#cursorS) low = middle;
      else high = middle - 1;
    }
    // At a forward step's shared boundary, keep the completed move highlighted.
    if (this.#stepTargetS !== null && this.#direction > 0 && this.#phase === "paused") {
      while (low > 0 && Math.abs(this.#samples[low - 1]!.timeS - this.#cursorS) < 1e-9) low--;
    }
    return this.#samples[low]!;
  }

  #sampleIndexAtCursor(): number {
    const sample = this.#sampleAtCursor();
    return sample ? this.#samples.indexOf(sample) : -1;
  }

  #clear(): void {
    if (this.#handle !== null) this.#deps.cancel(this.#handle);
    this.#handle = null;
  }

  #emit(): void {
    const index = this.#sampleIndexAtCursor();
    this.#deps.onChange({
      phase: this.#phase,
      cursorS: this.#cursorS,
      durationS: this.durationS,
      sampleIndex: index,
      torchOn: index >= 0 ? (this.#torch[index] ?? false) : false,
      rate: this.#rate,
    });
  }
}
