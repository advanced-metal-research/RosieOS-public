import assert from "node:assert/strict";
import test from "node:test";
import { jointMotionProfile, linearMotionProfile, tcpMotionProfile } from "../src/domain/motion-profile.ts";
import { createBlankSpreadsheetProject, createSpreadsheetRow, updateSpreadsheetRows } from "../src/domain/spreadsheet.ts";
import { fromRobotV4Program, toRobotV4Program } from "../src/adapters/robot-v4.ts";
import {
  lateralProfile,
  normalizeWeavePattern,
  rowTorchSettings,
  rowWeaveSettings,
  setRowArc,
  synchronizeTorchOrientations,
  torchGuideVectors,
  weavePreviewPoints,
} from "../src/domain/weld.ts";

test("torch guide vectors follow work and travel angle conventions", () => {
  // Travel along +X; zero angles keep the torch vertical.
  const vertical = torchGuideVectors([1, 0, 0], {
    workAngleDeg: 0,
    travelAngleDeg: 0,
    spinAngleDeg: 0,
  });
  assert.ok(vertical);
  assert.ok(Math.abs(vertical.torchUp[2] - 1) < 1e-9);

  // A pure work angle tilts across the seam, never along travel.
  const work = torchGuideVectors([1, 0, 0], {
    workAngleDeg: 45,
    travelAngleDeg: 0,
    spinAngleDeg: 0,
  });
  assert.ok(work);
  assert.ok(Math.abs(work.torchUp[0]) < 1e-9);
  assert.ok(Math.abs(Math.abs(work.torchUp[1]) - Math.SQRT1_2) < 1e-9);
  assert.ok(Math.abs(work.torchUp[2] - Math.SQRT1_2) < 1e-9);

  // A pure travel angle tilts along the travel direction only.
  const travel = torchGuideVectors([1, 0, 0], {
    workAngleDeg: 0,
    travelAngleDeg: 15,
    spinAngleDeg: 0,
  });
  assert.ok(travel);
  assert.ok(Math.abs(travel.torchUp[1]) < 1e-9);
  assert.ok(Math.abs(travel.torchUp[0]) > 1e-3);

  // Degenerate travel direction is rejected instead of guessed.
  assert.equal(
    torchGuideVectors([0, 0, 0], { workAngleDeg: 0, travelAngleDeg: 0, spinAngleDeg: 0 }),
    null,
  );
});

test("near-vertical tangents require a meaningful relative horizontal component", () => {
  const torch = { workAngleDeg: 0, travelAngleDeg: 0, spinAngleDeg: 0 };
  assert.equal(torchGuideVectors([1e-8, 0, 1], torch), null);
  assert.ok(torchGuideVectors([2e-6, 0, 1], torch));
  assert.equal(torchGuideVectors([1e-5, 0, 100], torch), null);
});

test("single weld row derives its tangent from the approach to retract chord", () => {
  const project = createBlankSpreadsheetProject();
  const approach = createSpreadsheetRow(project, "motion", "approach");
  const weld = createSpreadsheetRow(project, "motion", "weld");
  const retract = createSpreadsheetRow(project, "motion", "retract");
  for (const row of [approach, weld, retract]) {
    row.seamId = "single";
    row.sourceState = "cad_derived";
  }
  approach.phase = "approach";
  approach.targetTcpPose = [-0.05, 0, 0, 0, 0, 0];
  weld.phase = "weld";
  weld.targetTcpPose = [0, 0, 0, 0, 0, 0];
  weld.torchWorkAngleDeg = 45;
  weld.torchTravelAngleDeg = 10;
  retract.phase = "retract";
  retract.targetTcpPose = [0.05, 0, 0, 0, 0, 0];
  synchronizeTorchOrientations([approach, weld, retract]);
  assert.notDeepEqual(weld.targetTcpPose.slice(3), [0, 0, 0]);
  assert.deepEqual(approach.targetTcpPose.slice(3), weld.targetTcpPose.slice(3));
  assert.deepEqual(retract.targetTcpPose.slice(3), weld.targetTcpPose.slice(3));
});

test("seam synchronization unwraps the azimuth branch cut componentwise", () => {
  const project = createBlankSpreadsheetProject();
  const rows = [-15, -10, -5, 0, 5].map((degrees, index) => {
    const row = createSpreadsheetRow(project, "motion", `curve-${index}`);
    const angle = degrees * Math.PI / 180;
    row.seamId = "curve";
    row.phase = "weld";
    row.sourceState = "cad_derived";
    row.targetTcpPose = [Math.cos(angle), Math.sin(angle), 0, 0, 0, 0];
    row.currentTcpPose = [...row.targetTcpPose];
    row.torchWorkAngleDeg = 45;
    row.torchTravelAngleDeg = 10;
    return row;
  });
  synchronizeTorchOrientations(rows);
  for (let index = 1; index < rows.length; index += 1) {
    rows[index]!.targetTcpPose.slice(3).forEach((value, axis) => {
      assert.ok(
        Math.abs(value - rows[index - 1]!.targetTcpPose[axis + 3]!) <= Math.PI,
      );
    });
  }
  assert.ok(rows[2]!.targetTcpPose[5] > Math.PI);
});

test("weave preview stays on the seam envelope and respects disable states", () => {
  const straightSeam: Array<[number, number, number]> = [
    [0, 0, 0],
    [0.1, 0, 0],
  ];
  const points = weavePreviewPoints({
    pathPointsM: straightSeam,
    travelSpeedMps: 0.01,
    weave: { pattern: "zigzag", amplitudeMm: 3, frequencyHz: 1, dwellMs: 0 },
  });
  assert.ok(points.length > 20);
  const maxLateralM = Math.max(...points.map((point) => Math.abs(point[1])));
  assert.ok(maxLateralM <= 0.003 + 1e-9);
  assert.ok(maxLateralM > 0.002);
  // The ribbon spans the full seam.
  assert.ok(Math.abs(points[0]![0]) < 1e-9);
  assert.ok(Math.abs(points.at(-1)![0] - 0.1) < 1e-6);

  assert.deepEqual(
    weavePreviewPoints({
      pathPointsM: straightSeam,
      travelSpeedMps: 0.01,
      weave: { pattern: "none", amplitudeMm: 3, frequencyHz: 1, dwellMs: 0 },
    }),
    [],
  );
  assert.deepEqual(
    weavePreviewPoints({
      pathPointsM: straightSeam,
      travelSpeedMps: 0,
      weave: { pattern: "zigzag", amplitudeMm: 3, frequencyHz: 1, dwellMs: 0 },
    }),
    [],
  );
});

test("weave lateral profiles remain inside [-1, 1] for every pattern", () => {
  for (const pattern of ["zigzag", "circular", "trapezoidal"] as const) {
    for (let step = 0; step <= 40; step += 1) {
      const value = lateralProfile(pattern, step / 40 * 2);
      assert.ok(value >= -1 - 1e-9 && value <= 1 + 1e-9, `${pattern} at ${step}`);
    }
  }
  assert.equal(lateralProfile("none", 0.3), 0);
  assert.equal(normalizeWeavePattern("sawtooth"), "none");
});

test("torch and weave settings survive the Robot V4 wire round trip", () => {
  let project = createBlankSpreadsheetProject({ name: "Weld ext round trip" });
  const second = createSpreadsheetRow(project, "motion");
  project.rows.push(second);
  project = updateSpreadsheetRows(project, [second.id], (row) => {
    setRowArc(row, true);
    row.weldProcess = "gmaw";
    row.torchWorkAngleDeg = 40;
    row.torchTravelAngleDeg = -12;
    row.torchSpinAngleDeg = 5;
    row.weavePattern = "trapezoidal";
    row.weaveAmplitudeMm = 2.5;
    row.weaveFrequencyHz = 1.8;
    row.weaveDwellMs = 120;
  });

  const wire = toRobotV4Program(project);
  const wireRow = wire.rows.find((candidate) => candidate.row_id === second.id)!;
  assert.equal(wireRow.weld_enable, true);
  assert.equal(wireRow.arc_state, "on");
  assert.equal(wireRow.revpi_io_action, "weld_on");
  const packed = wireRow.extensions?.gradient_weld_v1 as Record<string, unknown>;
  assert.equal(packed.torch_work_angle_deg, 40);
  assert.equal(packed.weave_pattern, "trapezoidal");

  const restored = fromRobotV4Program(wire);
  const restoredRow = restored.rows.find((candidate) => candidate.id === second.id)!;
  assert.deepEqual(rowTorchSettings(restoredRow), {
    workAngleDeg: 40,
    travelAngleDeg: -12,
    spinAngleDeg: 5,
  });
  assert.deepEqual(rowWeaveSettings(restoredRow), {
    pattern: "trapezoidal",
    amplitudeMm: 2.5,
    frequencyHz: 1.8,
    dwellMs: 120,
  });
  // The packed block is unpacked into typed fields, not duplicated.
  assert.equal(restoredRow.extensions?.gradient_weld_v1, undefined);
});

test("linear motion profile chooses trapezoid or triangle correctly", () => {
  const trapezoid = linearMotionProfile(0.2, 0.05, 0.75);
  assert.equal(trapezoid.shape, "trapezoid");
  assert.ok(Math.abs(trapezoid.peakSpeedMps - 0.05) < 1e-12);
  assert.ok(trapezoid.cruiseDistanceM > 0.19);
  const expectedDuration = 0.05 / 0.75 * 2 + trapezoid.cruiseDistanceM / 0.05;
  assert.ok(Math.abs(trapezoid.durationS - expectedDuration) < 1e-9);

  const triangle = linearMotionProfile(0.001, 0.5, 0.75);
  assert.equal(triangle.shape, "triangle");
  assert.ok(triangle.peakSpeedMps < 0.5);
  assert.ok(Math.abs(triangle.accelDistanceM - 0.0005) < 1e-12);

  assert.equal(linearMotionProfile(0, 0.05, 0.75).shape, "unavailable");
});


test("joint profile uses actual sample times, signed velocity, and independent joint derivatives", () => {
  const samples = [0, 0.1, 0.3, 0.4].map(tS => ({ tS,
    qdRadS: new Float64Array([2 * tS, -4 * tS, 0]) }));
  const result = jointMotionProfile(samples)!;
  assert.equal(result.durationS, 0.4);
  assert.deepEqual(result.timesS, [0, 0.1, 0.3, 0.4]);
  assert.equal(result.velocityDegS[1]![2], -1.2 * 180 / Math.PI);
  for (const value of result.accelerationDegS2[0]!) assert.ok(Math.abs(value - 360 / Math.PI) < 1e-10);
  for (const value of result.accelerationDegS2[1]!) assert.ok(Math.abs(value + 720 / Math.PI) < 1e-10);
  assert.deepEqual(result.accelerationDegS2[2], [0, 0, 0, 0]);
  assert.equal(jointMotionProfile([]), null);
  assert.equal(jointMotionProfile([samples[0]!, samples[0]!]), null);
  assert.equal(jointMotionProfile([{ tS: 0, qdRadS: [NaN] }, { tS: 1, qdRadS: [0] }]), null);
});

test("TCP profile uses path distance in mm and sample timing, with rest endpoints", () => {
  const samples = [0, 0.1, 0.3, 0.4].map((tS, i) => ({ tS, qdRadS: [i === 0 || i === 3 ? 0 : 1] }));
  const result = tcpMotionProfile(samples, [[0, 0, 0], [0.003, 0.004, 0], [0.003, 0.024, 0], [0.006, 0.028, 0]])!;
  assert.deepEqual(result.timesS, [0, 0.05, 0.2, 0.35, 0.4]);
  for (const [i, expected] of [0, 50, 100, 50, 0].entries()) assert.ok(Math.abs(result.speedMmS[i]! - expected) < 1e-10);
  assert.ok(result.accelerationMmS2[0]! > 0);
  assert.ok(result.accelerationMmS2.at(-1)! < 0);
  const slow = tcpMotionProfile(samples.map(s => ({ ...s, tS: s.tS * 10 })), [[0, 0, 0], [0.003, 0.004, 0], [0.003, 0.024, 0], [0.006, 0.028, 0]])!;
  result.speedMmS.forEach((v, i) => assert.ok(Math.abs(slow.speedMmS[i]! - v / 10) < 1e-10));
  result.accelerationMmS2.forEach((v, i) => assert.ok(Math.abs(slow.accelerationMmS2[i]! - v / 100) < 1e-10));
  assert.equal(tcpMotionProfile(samples, []), null);
  assert.equal(tcpMotionProfile([samples[0]!, samples[0]!], [[0, 0, 0], [1, 0, 0]]), null);
});
