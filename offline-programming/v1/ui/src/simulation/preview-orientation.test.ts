// Prereg (red-first) suite for P2.10: the CONSUMER contract.
//
// so3.test.ts pins the math. This file pins that every preview-side consumer
// that interpolates a sampled orientation actually routes through it, so the
// operator's screen shows the SO(3) geodesic the Motion Server executes:
//
//   domain/playback.ts previewTrajectory  — row -> row, builds the preview
//   domain/playback.ts sampleAtTime       — sample -> sample, playback scrub
//   simulation/timeline.ts interpolateSample / sampleTimeline — sample -> sample
//
// The measured consequence these cases exist to prevent: at a reorienting
// segment boundary the accepted plan legally carries a 2*pi RPY representation
// gap between ADJACENT samples, which a componentwise lerp renders as a
// full-turn torch spin the robot never performs.

import test from "node:test";
import assert from "node:assert/strict";

import type {
  AuthoredProgramRow,
  JointVector,
  PlanTrajectorySample,
  TcpPose,
} from "../domain/project.ts";
import { previewTrajectory, sampleAtTime } from "../domain/playback.ts";
import {
  SO3_GEODESIC_TOLERANCE_RAD,
  type Rpy3,
  so3InterpolateRpy,
  so3RpyGeodesicAngleRad,
} from "../domain/so3.ts";
import type { SimulationSample } from "./contracts.ts";
import { interpolateSample, sampleTimeline } from "./timeline.ts";

const TWO_PI = Math.PI * 2;
const deg = (value: number): number => (value * 180) / Math.PI;
const rad = (degrees: number): number => (degrees * Math.PI) / 180;

/** Verbatim from motion-server/v1/tests/so3_interpolation_runtime_test.cpp. */
const PAIR24_FROM: Rpy3 = [-3.0212545394448225, 0.77031903290096937, 6.455119020278552];
const PAIR24_TO: Rpy3 = [0.0, 0.0, 6.2831853071795862];

/**
 * Adjacent samples that denote the SAME rotation but spell it a whole turn
 * apart — the representation gap a branch-crossing principal extraction emits.
 */
const GAP_FROM: Rpy3 = [rad(12), rad(-30), rad(170)];
const GAP_TO: Rpy3 = [rad(12), rad(-30), rad(170) + TWO_PI];

const rpyOf = (pose: readonly number[]): Rpy3 => [pose[3]!, pose[4]!, pose[5]!];

/**
 * Alpha-free geodesic-membership slack: a frame P on the SO(3) geodesic from A
 * to B satisfies d(A,P) + d(P,B) = d(A,B) exactly; any off-geodesic excursion
 * makes the sum strictly greater. Lets a consumer be checked without having to
 * replicate its internal sample spacing.
 */
function geodesicSlackRad(from: Rpy3, to: Rpy3, via: Rpy3): number {
  return so3RpyGeodesicAngleRad(from, via) +
    so3RpyGeodesicAngleRad(via, to) -
    so3RpyGeodesicAngleRad(from, to);
}

test("timeline interpolateSample keeps a 2*pi-gapped pair on the geodesic", () => {
  const left = simSample(0, [0, 0, 0, ...GAP_FROM]);
  const right = simSample(1, [0, 0, 0, ...GAP_TO]);
  let worst = 0;
  for (let step = 1; step <= 256; step += 1) {
    const interpolated = interpolateSample(left, right, step / 256);
    worst = Math.max(worst, so3RpyGeodesicAngleRad(GAP_FROM, rpyOf(interpolated.tcpPose!)));
  }
  assert.ok(
    worst <= SO3_GEODESIC_TOLERANCE_RAD,
    `preview swept ${deg(worst)} deg of visible rotation across a segment that ` +
      "denotes 0 deg — a full-turn torch spin the robot does not perform",
  );
});

test("timeline sampleTimeline scrubs pair-24 on the geodesic, via the shared module", () => {
  const samples = [
    simSample(0, [0.1, 0.2, 0.3, ...PAIR24_FROM]),
    simSample(1, [0.4, 0.2, 0.3, ...PAIR24_TO]),
  ];
  let worstSlack = 0;
  for (let step = 1; step < 256; step += 1) {
    const alpha = step / 256;
    const scrubbed = rpyOf(sampleTimeline(samples, alpha).sample!.tcpPose!);
    worstSlack = Math.max(worstSlack, geodesicSlackRad(PAIR24_FROM, PAIR24_TO, scrubbed));
    // Bit-exact agreement proves the scrub routes through the one shared
    // implementation rather than a second, separately-drifting copy.
    assert.deepEqual(
      scrubbed,
      so3InterpolateRpy(PAIR24_FROM, PAIR24_TO, alpha),
      `scrub at alpha=${alpha} does not match the shared SO(3) module`,
    );
  }
  assert.ok(
    worstSlack <= SO3_GEODESIC_TOLERANCE_RAD,
    `pair-24 scrub strayed ${deg(worstSlack)} deg off the geodesic`,
  );
});

test("timeline interpolation leaves translation and joints a componentwise lerp", () => {
  const left = simSample(0, [0.62, -0.19, 0.76, ...PAIR24_FROM], [0.1, 0.2, 0.3]);
  const right = simSample(1, [0.95, 0.19, 0.61, ...PAIR24_TO], [0.4, -0.2, 0.9]);
  for (const alpha of [0.125, 0.37, 0.5, 0.875]) {
    const interpolated = interpolateSample(left, right, alpha);
    for (let axis = 0; axis < 3; axis += 1) {
      const expected = left.tcpPose![axis]! +
        (right.tcpPose![axis]! - left.tcpPose![axis]!) * alpha;
      assert.equal(interpolated.tcpPose![axis], expected, `translation axis ${axis}`);
    }
    interpolated.jointsRad.forEach((value, index) => {
      const expected = left.jointsRad[index]! +
        (right.jointsRad[index]! - left.jointsRad[index]!) * alpha;
      assert.equal(value, expected, `joint ${index}`);
    });
  }
});

test("playback sampleAtTime keeps a 2*pi-gapped pair on the geodesic", () => {
  const trajectory: PlanTrajectorySample[] = [
    planSample(0, 0, [0, 0, 0, ...GAP_FROM]),
    planSample(1, 1, [0, 0, 0, ...GAP_TO]),
  ];
  let worst = 0;
  for (let step = 1; step <= 256; step += 1) {
    const { sample } = sampleAtTime(trajectory, step / 256);
    worst = Math.max(worst, so3RpyGeodesicAngleRad(GAP_FROM, rpyOf(sample!.tcpPose)));
  }
  assert.ok(
    worst <= SO3_GEODESIC_TOLERANCE_RAD,
    `playback scrub swept ${deg(worst)} deg across a segment that denotes 0 deg`,
  );
});

test("playback sampleAtTime scrubs pair-24 on the geodesic, via the shared module", () => {
  const trajectory: PlanTrajectorySample[] = [
    planSample(0, 0, [0.1, 0.2, 0.3, ...PAIR24_FROM]),
    planSample(1, 1, [0.4, 0.2, 0.3, ...PAIR24_TO]),
  ];
  let worstSlack = 0;
  for (let step = 1; step < 256; step += 1) {
    const alpha = step / 256;
    const scrubbed = rpyOf(sampleAtTime(trajectory, alpha).sample!.tcpPose);
    worstSlack = Math.max(worstSlack, geodesicSlackRad(PAIR24_FROM, PAIR24_TO, scrubbed));
    assert.deepEqual(
      scrubbed,
      so3InterpolateRpy(PAIR24_FROM, PAIR24_TO, alpha),
      `scrub at alpha=${alpha} does not match the shared SO(3) module`,
    );
  }
  assert.ok(
    worstSlack <= SO3_GEODESIC_TOLERANCE_RAD,
    `pair-24 playback scrub strayed ${deg(worstSlack)} deg off the geodesic`,
  );
});

test("previewTrajectory builds a 2*pi-gapped row pair on the geodesic", () => {
  const rows = [
    programRow("a", [0.62, -0.19, 0.76, ...GAP_FROM]),
    programRow("b", [0.95, 0.19, 0.61, ...GAP_TO]),
  ];
  const trajectory = previewTrajectory(rows);
  assert.ok(trajectory.length >= 3, "expected a multi-sample preview segment");
  let worst = 0;
  for (const sample of trajectory) {
    worst = Math.max(worst, so3RpyGeodesicAngleRad(GAP_FROM, rpyOf(sample.tcpPose)));
  }
  assert.ok(
    worst <= SO3_GEODESIC_TOLERANCE_RAD,
    `the generated preview swept ${deg(worst)} deg across a row pair that ` +
      "denotes 0 deg of reorientation",
  );
});

test("previewTrajectory builds pair-24 on the geodesic and lands on the row target", () => {
  const rows = [
    programRow("a", [0.62, -0.19, 0.76, ...PAIR24_FROM]),
    programRow("b", [0.95, 0.19, 0.61, ...PAIR24_TO]),
  ];
  const trajectory = previewTrajectory(rows);
  let worstSlack = 0;
  for (const sample of trajectory) {
    worstSlack = Math.max(
      worstSlack,
      geodesicSlackRad(PAIR24_FROM, PAIR24_TO, rpyOf(sample.tcpPose)),
    );
  }
  assert.ok(
    worstSlack <= SO3_GEODESIC_TOLERANCE_RAD,
    `the generated preview strayed ${deg(worstSlack)} deg off the geodesic`,
  );
  // The authored row frame must survive verbatim at both ends of the segment.
  assert.deepEqual(rpyOf(trajectory[0]!.tcpPose), [...PAIR24_FROM]);
  assert.deepEqual(rpyOf(trajectory.at(-1)!.tcpPose), [...PAIR24_TO]);
});

function simSample(
  timeS: number,
  tcpPose: number[],
  jointsRad: number[] = [0, 0, 0],
): SimulationSample {
  return {
    modelId: "rosie_1400_v3",
    jointOrder: ["j1", "j2", "j3"],
    jointsRad,
    source: "preview",
    timeS,
    tcpPose: tcpPose as TcpPose,
    motion: "travel",
    seamId: null,
    rowIndex: Math.round(timeS),
  };
}

function planSample(index: number, timeS: number, tcpPose: number[]): PlanTrajectorySample {
  return {
    index,
    rowIndex: index,
    seamId: null,
    phase: "travel",
    weldOn: false,
    timeS,
    tcpPose: tcpPose as TcpPose,
    jointPositionsRad: [0, 0, 0] as unknown as JointVector,
  };
}

function programRow(id: string, targetTcpPose: number[]): AuthoredProgramRow {
  return {
    id,
    enabled: true,
    label: id,
    command: "MOVE",
    moveType: "LINE",
    phase: "travel",
    seamId: null,
    passId: "",
    passRole: "none",
    toolFrameId: "torch_tcp",
    workFrameId: "world",
    fixtureId: null,
    targetTcpPose: targetTcpPose as TcpPose,
    targetJointsRad: [0, 0, 0, 0, 0, 0] as unknown as JointVector,
    positionerTargetsRad: [0, 0, 0],
    tcpVelocityMps: 0.05,
    jointVelocityRadS: 0.25,
    accelerationMps2: 0.75,
    jointAccelerationRadS2: 1,
    blendRadiusM: 0,
    fineStop: true,
    weldEnabled: false,
    arcState: "off",
    wireFeedMPerMin: 0,
    voltageV: 0,
    weldProfileId: 0,
    weaveProfileId: 0,
    weldProcess: "none",
    revPiIOSlot: null,
    revPiIOAction: "none",
    source: "offline_programming",
  };
}
