import { interpolateTcpPose } from "../domain/so3.ts";
import type {
  PlaybackSnapshot,
  PlaybackStatus,
  SimulationSample,
} from "./contracts.ts";

const TIME_EPSILON_S = 1e-9;

export interface TimelineValidation {
  ok: boolean;
  errors: string[];
}

export interface TimelineLookup<TModelId extends string = string> {
  sample: SimulationSample<TModelId> | null;
  index: number;
  done: boolean;
}

export function validateTimeline<TModelId extends string>(
  samples: readonly SimulationSample<TModelId>[],
): TimelineValidation {
  const errors: string[] = [];
  if (samples.length === 0) return { ok: true, errors };

  const first = samples[0]!;
  if (!first.modelId.trim()) errors.push("sample 0 modelId is required");
  if (first.jointOrder.length === 0) errors.push("sample 0 jointOrder is empty");

  for (let index = 0; index < samples.length; index += 1) {
    const sample = samples[index]!;
    if (!Number.isFinite(sample.timeS) || sample.timeS < 0) {
      errors.push(`sample ${index} timeS must be finite and non-negative`);
    }
    if (index > 0 && sample.timeS + TIME_EPSILON_S < samples[index - 1]!.timeS) {
      errors.push(`sample ${index} timeS is not monotonic`);
    }
    if (sample.modelId !== first.modelId) {
      errors.push(`sample ${index} modelId does not match sample 0`);
    }
    if (!sameStrings(sample.jointOrder, first.jointOrder)) {
      errors.push(`sample ${index} jointOrder does not match sample 0`);
    }
    if (sample.jointsRad.length !== first.jointOrder.length) {
      errors.push(`sample ${index} joint vector length does not match jointOrder`);
    }
    if (sample.jointsRad.some((value) => !Number.isFinite(value))) {
      errors.push(`sample ${index} contains a non-finite joint value`);
    }
    if (sample.tcpPose?.some((value) => !Number.isFinite(value))) {
      errors.push(`sample ${index} contains a non-finite TCP value`);
    }
  }

  return { ok: errors.length === 0, errors };
}

export function assertTimeline<TModelId extends string>(
  samples: readonly SimulationSample<TModelId>[],
): void {
  const validation = validateTimeline(samples);
  if (!validation.ok) {
    throw new Error(`invalid simulation timeline: ${validation.errors.join("; ")}`);
  }
}

export function interpolateSample<TModelId extends string>(
  left: SimulationSample<TModelId>,
  right: SimulationSample<TModelId>,
  alpha: number,
  timeS = lerp(left.timeS, right.timeS, clamp01(alpha)),
): SimulationSample<TModelId> {
  if (left.modelId !== right.modelId || !sameStrings(left.jointOrder, right.jointOrder)) {
    throw new Error("cannot interpolate samples with different model identity");
  }
  const t = clamp01(alpha);
  return {
    modelId: left.modelId,
    jointOrder: [...left.jointOrder],
    jointsRad: lerpArray(left.jointsRad, right.jointsRad, t),
    source: "playback",
    observedAtMs: undefined,
    timeS,
    // Orientation travels the SO(3) geodesic, matching the Motion Server
    // (robot_v4_cartesian_so3.hpp). A componentwise RPY lerp would render the
    // 2*pi representation gaps an accepted plan legally carries between
    // adjacent samples as a full-turn torch spin the robot never performs.
    tcpPose: left.tcpPose && right.tcpPose
      ? interpolateTcpPose(left.tcpPose, right.tcpPose, t) as unknown as SimulationSample<TModelId>["tcpPose"]
      : left.tcpPose ?? right.tcpPose,
    motion: t >= 1 ? right.motion : left.motion,
    seamId: t >= 0.5 ? right.seamId : left.seamId,
    rowIndex: t >= 0.5 ? right.rowIndex : left.rowIndex,
  };
}

export function sampleTimeline<TModelId extends string>(
  samples: readonly SimulationSample<TModelId>[],
  timeS: number,
): TimelineLookup<TModelId> {
  if (samples.length === 0) return { sample: null, index: -1, done: true };
  const first = samples[0]!;
  const last = samples[samples.length - 1]!;
  const cursor = Math.max(0, Number.isFinite(timeS) ? timeS : 0);
  if (samples.length === 1 || cursor >= last.timeS - TIME_EPSILON_S) {
    return { sample: cloneSample(last), index: samples.length - 1, done: true };
  }
  if (cursor <= first.timeS + TIME_EPSILON_S) {
    return { sample: cloneSample(first), index: 0, done: false };
  }

  let low = 1;
  let high = samples.length - 1;
  while (low < high) {
    const middle = Math.floor((low + high) / 2);
    if (samples[middle]!.timeS < cursor) low = middle + 1;
    else high = middle;
  }
  const rightIndex = low;
  const leftIndex = rightIndex - 1;
  const left = samples[leftIndex]!;
  const right = samples[rightIndex]!;
  const span = right.timeS - left.timeS;
  const alpha = span > TIME_EPSILON_S ? (cursor - left.timeS) / span : 1;
  return {
    sample: interpolateSample(left, right, alpha, cursor),
    index: leftIndex,
    done: false,
  };
}

export class PlaybackClock<TModelId extends string = string> {
  #samples: SimulationSample<TModelId>[] = [];
  #cursorS = 0;
  #rate = 1;
  #loop = false;
  #status: PlaybackStatus = "empty";

  get samples(): readonly SimulationSample<TModelId>[] {
    return this.#samples;
  }

  get durationS(): number {
    return this.#samples.at(-1)?.timeS ?? 0;
  }

  get status(): PlaybackStatus {
    return this.#status;
  }

  load(samples: readonly SimulationSample<TModelId>[]): PlaybackSnapshot<TModelId> {
    assertTimeline(samples);
    this.#samples = samples.map(cloneSample);
    this.#cursorS = 0;
    this.#status = this.#samples.length === 0 ? "empty" : "paused";
    return this.snapshot();
  }

  play(): PlaybackSnapshot<TModelId> {
    if (this.#samples.length === 0) {
      this.#status = "empty";
    } else if (this.durationS <= TIME_EPSILON_S) {
      this.#status = "ended";
    } else {
      if (this.#status === "ended") this.#cursorS = 0;
      this.#status = "playing";
    }
    return this.snapshot();
  }

  pause(): PlaybackSnapshot<TModelId> {
    if (this.#status === "playing") this.#status = "paused";
    return this.snapshot();
  }

  stop(): PlaybackSnapshot<TModelId> {
    this.#cursorS = 0;
    this.#status = this.#samples.length === 0 ? "empty" : "paused";
    return this.snapshot();
  }

  seek(timeS: number): PlaybackSnapshot<TModelId> {
    const requested = Number.isFinite(timeS) ? timeS : 0;
    this.#cursorS = clamp(requested, 0, this.durationS);
    if (this.#samples.length === 0) this.#status = "empty";
    else if (this.#cursorS >= this.durationS - TIME_EPSILON_S) this.#status = "ended";
    else if (this.#status === "ended") this.#status = "paused";
    return this.snapshot();
  }

  setPlaybackRate(rate: number): PlaybackSnapshot<TModelId> {
    if (!Number.isFinite(rate) || rate <= 0) {
      throw new Error("playback rate must be finite and greater than zero");
    }
    this.#rate = rate;
    return this.snapshot();
  }

  setLoop(loop: boolean): PlaybackSnapshot<TModelId> {
    this.#loop = loop;
    return this.snapshot();
  }

  advance(deltaS: number): PlaybackSnapshot<TModelId> {
    if (this.#status !== "playing") return this.snapshot();
    if (!Number.isFinite(deltaS) || deltaS < 0) {
      throw new Error("playback delta must be finite and non-negative");
    }

    const duration = this.durationS;
    const next = this.#cursorS + deltaS * this.#rate;
    if (this.#loop && duration > TIME_EPSILON_S) {
      this.#cursorS = next % duration;
    } else if (next >= duration - TIME_EPSILON_S) {
      this.#cursorS = duration;
      this.#status = "ended";
    } else {
      this.#cursorS = next;
    }
    return this.snapshot();
  }

  snapshot(): PlaybackSnapshot<TModelId> {
    const lookup = sampleTimeline(this.#samples, this.#cursorS);
    return {
      status: this.#status,
      cursorS: this.#cursorS,
      durationS: this.durationS,
      playbackRate: this.#rate,
      loop: this.#loop,
      sample: lookup.sample,
      sampleIndex: lookup.index,
    };
  }
}

function cloneSample<TModelId extends string>(
  sample: SimulationSample<TModelId>,
): SimulationSample<TModelId> {
  return {
    ...sample,
    jointOrder: [...sample.jointOrder],
    jointsRad: [...sample.jointsRad],
    tcpPose: sample.tcpPose ? [...sample.tcpPose] as unknown as SimulationSample<TModelId>["tcpPose"] : undefined,
  };
}

function lerpArray(left: readonly number[], right: readonly number[], alpha: number): number[] {
  const length = Math.max(left.length, right.length);
  return Array.from({ length }, (_, index) => {
    const a = left[index] ?? right[index] ?? 0;
    const b = right[index] ?? a;
    return lerp(a, b, alpha);
  });
}

function sameStrings(left: readonly string[], right: readonly string[]): boolean {
  return left.length === right.length && left.every((value, index) => value === right[index]);
}

function lerp(left: number, right: number, alpha: number): number {
  return left + (right - left) * alpha;
}

function clamp01(value: number): number {
  return clamp(Number.isFinite(value) ? value : 0, 0, 1);
}

function clamp(value: number, minimum: number, maximum: number): number {
  return Math.max(minimum, Math.min(maximum, value));
}
