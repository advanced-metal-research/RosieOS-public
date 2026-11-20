import type {
  PlaybackSnapshot,
  SimulationSample,
} from "./contracts.ts";
import { PlaybackClock } from "./timeline.ts";

export interface AnimationScheduler {
  now(): number;
  request(callback: (nowMs: number) => void): number;
  cancel(handle: number): void;
}

export type PlaybackListener<TModelId extends string = string> = (
  snapshot: PlaybackSnapshot<TModelId>,
) => void;

const browserScheduler: AnimationScheduler = {
  now: () => performance.now(),
  request: (callback) => requestAnimationFrame(callback),
  cancel: (handle) => cancelAnimationFrame(handle),
};

/** requestAnimationFrame driver around the deterministic PlaybackClock. */
export class TimedPlayback<TModelId extends string = string> {
  readonly clock = new PlaybackClock<TModelId>();
  #scheduler: AnimationScheduler;
  #listener: PlaybackListener<TModelId>;
  #frameHandle: number | null = null;
  #previousMs = 0;
  #disposed = false;

  constructor(
    listener: PlaybackListener<TModelId>,
    scheduler: AnimationScheduler = browserScheduler,
  ) {
    this.#listener = listener;
    this.#scheduler = scheduler;
  }

  load(samples: readonly SimulationSample<TModelId>[]): PlaybackSnapshot<TModelId> {
    this.assertActive();
    this.cancelFrame();
    return this.emit(this.clock.load(samples));
  }

  play(): PlaybackSnapshot<TModelId> {
    this.assertActive();
    const snapshot = this.emit(this.clock.play());
    if (snapshot.status === "playing" && this.#frameHandle === null) {
      this.#previousMs = this.#scheduler.now();
      this.#frameHandle = this.#scheduler.request(this.onFrame);
    }
    return snapshot;
  }

  pause(): PlaybackSnapshot<TModelId> {
    this.assertActive();
    this.cancelFrame();
    return this.emit(this.clock.pause());
  }

  stop(): PlaybackSnapshot<TModelId> {
    this.assertActive();
    this.cancelFrame();
    return this.emit(this.clock.stop());
  }

  seek(timeS: number): PlaybackSnapshot<TModelId> {
    this.assertActive();
    return this.emit(this.clock.seek(timeS));
  }

  setPlaybackRate(rate: number): PlaybackSnapshot<TModelId> {
    this.assertActive();
    return this.emit(this.clock.setPlaybackRate(rate));
  }

  setLoop(loop: boolean): PlaybackSnapshot<TModelId> {
    this.assertActive();
    return this.emit(this.clock.setLoop(loop));
  }

  dispose(): void {
    if (this.#disposed) return;
    this.cancelFrame();
    this.#disposed = true;
  }

  private onFrame = (nowMs: number): void => {
    this.#frameHandle = null;
    if (this.#disposed) return;
    const deltaS = Math.max(0, Math.min(0.25, (nowMs - this.#previousMs) / 1000));
    this.#previousMs = nowMs;
    const snapshot = this.emit(this.clock.advance(deltaS));
    if (snapshot.status === "playing") {
      this.#frameHandle = this.#scheduler.request(this.onFrame);
    }
  };

  private emit(snapshot: PlaybackSnapshot<TModelId>): PlaybackSnapshot<TModelId> {
    this.#listener(snapshot);
    return snapshot;
  }

  private cancelFrame(): void {
    if (this.#frameHandle === null) return;
    this.#scheduler.cancel(this.#frameHandle);
    this.#frameHandle = null;
  }

  private assertActive(): void {
    if (this.#disposed) throw new Error("timed playback has been disposed");
  }
}
