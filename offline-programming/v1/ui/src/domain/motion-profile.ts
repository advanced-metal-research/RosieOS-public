/**
 * Estimated linear motion profile for one spreadsheet move. This is a
 * planning aid rendered in the row inspector; the Motion Server/Tesseract
 * accepted plan remains the only timing authority.
 */
export interface LinearMotionProfile {
  shape: "trapezoid" | "triangle" | "unavailable";
  distanceM: number;
  targetSpeedMps: number;
  peakSpeedMps: number;
  accelerationMps2: number;
  accelDistanceM: number;
  cruiseDistanceM: number;
  decelDistanceM: number;
  durationS: number;
}

export function linearMotionProfile(
  distanceM: number,
  targetSpeedMps: number,
  accelerationMps2: number,
): LinearMotionProfile {
  const distance = finite(distanceM);
  const speed = finite(targetSpeedMps);
  const acceleration = finite(accelerationMps2);
  if (distance <= 0 || speed <= 0) {
    return {
      shape: "unavailable",
      distanceM: Math.max(0, distance),
      targetSpeedMps: Math.max(0, speed),
      peakSpeedMps: 0,
      accelerationMps2: Math.max(0, acceleration),
      accelDistanceM: 0,
      cruiseDistanceM: 0,
      decelDistanceM: 0,
      durationS: 0,
    };
  }
  // Without an authored acceleration, estimate one that reaches the target
  // speed in roughly one second — the same approximation GradientOS used.
  const accel = acceleration > 0 ? acceleration : speed;
  const rampDistanceM = speed * speed / (2 * accel);
  if (rampDistanceM * 2 <= distance) {
    const cruiseDistanceM = distance - rampDistanceM * 2;
    return {
      shape: "trapezoid",
      distanceM: distance,
      targetSpeedMps: speed,
      peakSpeedMps: speed,
      accelerationMps2: accel,
      accelDistanceM: rampDistanceM,
      cruiseDistanceM,
      decelDistanceM: rampDistanceM,
      durationS: speed / accel * 2 + cruiseDistanceM / speed,
    };
  }
  // Too short to reach the requested speed: triangular profile.
  const peakSpeedMps = Math.sqrt(distance * accel);
  const half = distance / 2;
  return {
    shape: "triangle",
    distanceM: distance,
    targetSpeedMps: speed,
    peakSpeedMps,
    accelerationMps2: accel,
    accelDistanceM: half,
    cruiseDistanceM: 0,
    decelDistanceM: half,
    durationS: peakSpeedMps / accel * 2,
  };
}

export interface JointMotionProfile {
  durationS: number;
  timesS: number[];
  velocityDegS: number[][];
  accelerationDegS2: number[][];
}

export interface TcpMotionProfile {
  timesS: number[];
  speedMmS: number[];
  accelerationMmS2: number[];
}

/** Translational speed from consecutive FK positions, at interval midpoints.
 * The endpoint velocities tell us when the planned move is at rest. */
export function tcpMotionProfile(
  samples: readonly { tS: number; qdRadS: ArrayLike<number> }[],
  positionsM: readonly (readonly number[])[],
): TcpMotionProfile | null {
  if (samples.length < 2 || samples.length !== positionsM.length ||
      positionsM.some(p => p.length !== 3 || !p.every(Number.isFinite))) return null;
  const start = samples[0]!.tS;
  const timesS = [0], speedMmS: number[] = [];
  for (let i = 1; i < samples.length; i++) {
    const dt = samples[i]!.tS - samples[i - 1]!.tS;
    if (!Number.isFinite(dt) || dt <= 0) return null;
    timesS.push((samples[i]!.tS + samples[i - 1]!.tS) / 2 - start);
    speedMmS.push(Math.hypot(...positionsM[i]!.map((v, axis) => v - positionsM[i - 1]![axis]!)) * 1000 / dt);
  }
  const stopped = (i: number) => Array.from(samples[i]!.qdRadS).every(v => v === 0);
  speedMmS.unshift(stopped(0) ? 0 : speedMmS[0]!);
  speedMmS.push(stopped(samples.length - 1) ? 0 : speedMmS.at(-1)!);
  timesS.push(samples.at(-1)!.tS - start);
  const accelerationMmS2 = speedMmS.map((_, i) => {
    const a = Math.max(0, i - 1), b = Math.min(speedMmS.length - 1, i + 1);
    return (speedMmS[b]! - speedMmS[a]!) / (timesS[b]! - timesS[a]!);
  });
  return { timesS, speedMmS, accelerationMmS2 };
}

/** Display data from the execution samples. Acceleration is the interval
 * derivative of velocity, not a separately invented motion profile. */
export function jointMotionProfile(samples: readonly {
  tS: number; qdRadS: ArrayLike<number>;
}[]): JointMotionProfile | null {
  if (samples.length < 2) return null;
  const start = samples[0]!.tS;
  const timesS = samples.map(sample => sample.tS - start);
  if (timesS.some((time, i) => !Number.isFinite(time) || (i > 0 && time <= timesS[i - 1]!))) return null;
  const axes = samples[0]!.qdRadS.length;
  if (!axes || samples.some(sample => sample.qdRadS.length !== axes || Array.from(sample.qdRadS).some(v => !Number.isFinite(v)))) return null;
  const velocityDegS = Array.from({ length: axes }, (_, axis) => samples.map(s => s.qdRadS[axis]! * 180 / Math.PI));
  const accelerationDegS2 = velocityDegS.map(values => values.map((_, i) => {
    const a = i === values.length - 1 ? i - 1 : i;
    return (values[a + 1]! - values[a]!) / (timesS[a + 1]! - timesS[a]!);
  }));
  return { durationS: timesS.at(-1)!, timesS, velocityDegS, accelerationDegS2 };
}

function finite(value: unknown): number {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : 0;
}
