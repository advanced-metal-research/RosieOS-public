// Prereg (red-first) suite for P2.10: SO(3) orientation interpolation in the
// BROWSER PREVIEW. Every numeric case is lifted from the Motion Server's
// reference suite, motion-server/v1/tests/so3_interpolation_runtime_test.cpp,
// so the preview is pinned to the same path the robot executes rather than to
// its own independent notion of "close enough".
//
// Written against the geodesic bound BEFORE the fix exists: while
// so3InterpolateRpy still carries its STAGE-2 componentwise Euler lerp these
// cases fail, and they pass once it slerps.

import test from "node:test";
import assert from "node:assert/strict";
import * as THREE from "three";

import {
  SO3_GEODESIC_TOLERANCE_RAD,
  type Mat3,
  type Rpy3,
  type So3Quaternion,
  interpolateTcpPose,
  so3InterpolateRpy,
  so3QuaternionFromRpy,
  so3QuaternionRotationMatrix,
  so3QuaternionSlerp,
  so3RotationGeodesicAngleRad,
  so3RpyGeodesicAngleRad,
  so3RpyRotationMatrix,
  so3UnwrapAngleToward,
} from "./so3.ts";

const TWO_PI = Math.PI * 2;
const deg = (rad: number): number => (rad * 180) / Math.PI;
const rad = (degrees: number): number => (degrees * Math.PI) / 180;

// ---------------------------------------------------------------------------
// Independent references. Deliberately NOT reusing so3.ts helpers, so a
// convention error in the module cannot hide by symmetry. Ported from the
// same-named helpers in the C++ suite.
// ---------------------------------------------------------------------------

/** R = Rz(yaw) * Ry(pitch) * Rx(roll), as an explicit triple product. */
function referenceRpyMatrix(rpy: Rpy3): Mat3 {
  const [cr, sr] = [Math.cos(rpy[0]), Math.sin(rpy[0])];
  const [cp, sp] = [Math.cos(rpy[1]), Math.sin(rpy[1])];
  const [cy, sy] = [Math.cos(rpy[2]), Math.sin(rpy[2])];
  const rx: Mat3 = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]];
  const ry: Mat3 = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]];
  const rz: Mat3 = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]];
  return multiply(multiply(rz, ry), rx);
}

function multiply(a: Mat3, b: Mat3): Mat3 {
  const out = zeros();
  for (let r = 0; r < 3; r += 1) {
    for (let c = 0; c < 3; c += 1) {
      let sum = 0;
      for (let k = 0; k < 3; k += 1) sum += a[r]![k]! * b[k]![c]!;
      out[r]![c] = sum;
    }
  }
  return out;
}

function zeros(): Mat3 {
  return [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
}

/**
 * Geodesic sample at `alpha` via the matrix exponential of log(Ra^T Rb): an
 * independent construction of the ground-truth path, not built from quaternions.
 */
function referenceGeodesicSample(fromRpy: Rpy3, toRpy: Rpy3, alpha: number): Mat3 {
  const ra = referenceRpyMatrix(fromRpy);
  const rb = referenceRpyMatrix(toRpy);
  const relative = zeros();
  for (let r = 0; r < 3; r += 1) {
    for (let c = 0; c < 3; c += 1) {
      let sum = 0;
      for (let k = 0; k < 3; k += 1) sum += ra[k]![r]! * rb[k]![c]!;
      relative[r]![c] = sum;
    }
  }
  const trace = relative[0]![0]! + relative[1]![1]! + relative[2]![2]!;
  const angle = Math.acos(Math.max(-1, Math.min(1, (trace - 1) / 2)));
  const sinAngle = Math.sin(angle);
  const axis = Math.abs(sinAngle) > 1.0e-12
    ? [
      (relative[2]![1]! - relative[1]![2]!) / (2 * sinAngle),
      (relative[0]![2]! - relative[2]![0]!) / (2 * sinAngle),
      (relative[1]![0]! - relative[0]![1]!) / (2 * sinAngle),
    ]
    : [0, 0, 0];
  const t = alpha * angle;
  const c = Math.cos(t);
  const s = Math.sin(t);
  const k: Mat3 = [
    [0, -axis[2]!, axis[1]!],
    [axis[2]!, 0, -axis[0]!],
    [-axis[1]!, axis[0]!, 0],
  ];
  const kk = multiply(k, k);
  const step = zeros();
  for (let r = 0; r < 3; r += 1) {
    for (let c2 = 0; c2 < 3; c2 += 1) {
      step[r]![c2] = (r === c2 ? 1 : 0) + s * k[r]![c2]! + (1 - c) * kk[r]![c2]!;
    }
  }
  return multiply(ra, step);
}

/**
 * Angle between the torch axes of two frames. Local +X is the torch direction
 * (weld.ts torchOrientationRpy), i.e. the first column of R.
 */
function torchAxisAngleRad(a: Mat3, b: Mat3): number {
  let dot = 0;
  for (let r = 0; r < 3; r += 1) dot += a[r]![0]! * b[r]![0]!;
  return Math.acos(Math.max(-1, Math.min(1, dot)));
}

interface Excursion {
  worstOrientationRad: number;
  worstTorchAxisRad: number;
  worstAlpha: number;
}

/** Sweep a segment and measure how far the interpolated path strays. */
function measureExcursion(fromRpy: Rpy3, toRpy: Rpy3, samples: number): Excursion {
  const worst: Excursion = {
    worstOrientationRad: 0,
    worstTorchAxisRad: 0,
    worstAlpha: 0,
  };
  for (let sample = 1; sample <= samples; sample += 1) {
    const alpha = sample / samples;
    const actual = so3RpyRotationMatrix(so3InterpolateRpy(fromRpy, toRpy, alpha));
    const expected = referenceGeodesicSample(fromRpy, toRpy, alpha);
    const orientation = so3RotationGeodesicAngleRad(actual, expected);
    if (orientation > worst.worstOrientationRad) {
      worst.worstOrientationRad = orientation;
      worst.worstAlpha = alpha;
    }
    worst.worstTorchAxisRad = Math.max(
      worst.worstTorchAxisRad,
      torchAxisAngleRad(actual, expected),
    );
  }
  return worst;
}

// ---------------------------------------------------------------------------
// The measured case, verbatim from the C++ suite (kPair24From / kPair24To).
//
//   a = top-left seam retract row  (unwrapped: yaw = 369.851 deg, |yaw| > pi)
//   b = front-left seam approach row
//
//   componentwise |delta| = [173.1051, 44.1360, 9.8511] deg
//   true geodesic         = 177.3273 deg
//   old Euler-lerp worst  =  21.2203 deg orientation / 21.2019 deg torch axis
// ---------------------------------------------------------------------------
const PAIR24_FROM: Rpy3 = [
  -3.0212545394448225, // roll  -173.1051339449 deg
  0.77031903290096937, // pitch   44.1360294638 deg
  6.455119020278552, //   yaw    369.8510761166 deg
];
const PAIR24_TO: Rpy3 = [
  0.0, //                 roll     0 deg
  0.0, //                 pitch    0 deg
  6.2831853071795862, //  yaw    360 deg
];

test("RPY convention matches an explicit Rz*Ry*Rx product and the quaternion path", () => {
  let worstMatrix = 0;
  let worstQuaternion = 0;
  for (let i = -9; i <= 9; i += 1) {
    for (let j = -9; j <= 9; j += 1) {
      for (let k = -9; k <= 9; k += 1) {
        const rpy: Rpy3 = [i * 0.7, j * 0.5, k * 0.9];
        const header = so3RpyRotationMatrix(rpy);
        const reference = referenceRpyMatrix(rpy);
        for (let r = 0; r < 3; r += 1) {
          for (let c = 0; c < 3; c += 1) {
            worstMatrix = Math.max(
              worstMatrix,
              Math.abs(header[r]![c]! - reference[r]![c]!),
            );
          }
        }
        worstQuaternion = Math.max(
          worstQuaternion,
          so3RotationGeodesicAngleRad(
            header,
            so3QuaternionRotationMatrix(so3QuaternionFromRpy(rpy)),
          ),
        );
      }
    }
  }
  assert.ok(worstMatrix <= 1.0e-15, `worst entry delta ${worstMatrix}`);
  assert.ok(
    worstQuaternion <= SO3_GEODESIC_TOLERANCE_RAD,
    `worst geodesic delta ${worstQuaternion} rad`,
  );
});

test("the preview's RPY convention is the one the viewer displays (Three.js ZYX)", () => {
  // applyTcpPose (viewer/overlays.ts) renders tcpPose[3..5] as Euler order
  // "ZYX". If that is not Rz(yaw)*Ry(pitch)*Rx(roll) then interpolating on the
  // geodesic in this convention would still draw the wrong frame.
  let worst = 0;
  for (let i = -4; i <= 4; i += 1) {
    for (let j = -4; j <= 4; j += 1) {
      for (let k = -4; k <= 4; k += 1) {
        const rpy: Rpy3 = [i * 0.8, j * 0.6, k * 1.1];
        const three = new THREE.Matrix4().makeRotationFromEuler(
          new THREE.Euler(rpy[0], rpy[1], rpy[2], "ZYX"),
        );
        const e = three.elements; // column-major
        const asMat3: Mat3 = [
          [e[0]!, e[4]!, e[8]!],
          [e[1]!, e[5]!, e[9]!],
          [e[2]!, e[6]!, e[10]!],
        ];
        worst = Math.max(
          worst,
          so3RotationGeodesicAngleRad(so3RpyRotationMatrix(rpy), asMat3),
        );
      }
    }
  }
  assert.ok(worst <= SO3_GEODESIC_TOLERANCE_RAD, `worst delta ${worst} rad`);
});

test("pair-24 interpolates on the geodesic instead of excursing 21 deg", () => {
  const trueAngle = so3RpyGeodesicAngleRad(PAIR24_FROM, PAIR24_TO);
  assert.ok(
    Math.abs(deg(trueAngle) - 177.3273) < 1.0e-3,
    `true reorientation measured ${deg(trueAngle)} deg`,
  );

  const worst = measureExcursion(PAIR24_FROM, PAIR24_TO, 2000);
  assert.ok(
    worst.worstOrientationRad <= SO3_GEODESIC_TOLERANCE_RAD,
    `worst orientation deviation ${worst.worstOrientationRad} rad ` +
      `(${deg(worst.worstOrientationRad)} deg) @alpha=${worst.worstAlpha}`,
  );
  assert.ok(
    worst.worstTorchAxisRad <= SO3_GEODESIC_TOLERANCE_RAD,
    `worst torch-axis deviation ${worst.worstTorchAxisRad} rad ` +
      `(${deg(worst.worstTorchAxisRad)} deg)`,
  );
});

test("pair-24 progress is monotone and bounded by the total rotation", () => {
  const total = so3RpyGeodesicAngleRad(PAIR24_FROM, PAIR24_TO);
  let previousDistance = 0;
  let worstRegression = 0;
  for (let sample = 1; sample <= 512; sample += 1) {
    const alpha = sample / 512;
    const distance = so3RpyGeodesicAngleRad(
      PAIR24_FROM,
      so3InterpolateRpy(PAIR24_FROM, PAIR24_TO, alpha),
    );
    worstRegression = Math.max(worstRegression, previousDistance - distance);
    previousDistance = distance;
    if (distance > total + SO3_GEODESIC_TOLERANCE_RAD) {
      worstRegression = Math.max(worstRegression, distance - total);
    }
  }
  assert.ok(
    worstRegression <= SO3_GEODESIC_TOLERANCE_RAD,
    `worst regression/overshoot ${worstRegression} rad against total ${total} rad`,
  );
});

test("a yaw +/-180 deg wrap takes the 2 deg shortest arc, not 358 deg the wrong way", () => {
  const from: Rpy3 = [0, 0, rad(179)];
  const to: Rpy3 = [0, 0, rad(-179)];
  const trueAngle = so3RpyGeodesicAngleRad(from, to);
  assert.ok(
    Math.abs(deg(trueAngle) - 2) < 1.0e-9,
    `true reorientation measured ${deg(trueAngle)} deg`,
  );

  const worst = measureExcursion(from, to, 512);
  assert.ok(
    worst.worstOrientationRad <= SO3_GEODESIC_TOLERANCE_RAD,
    `worst deviation ${deg(worst.worstOrientationRad)} deg`,
  );

  let previous: Rpy3 = from;
  let traversed = 0;
  for (let sample = 1; sample <= 512; sample += 1) {
    const interpolated = so3InterpolateRpy(from, to, sample / 512);
    traversed += so3RpyGeodesicAngleRad(previous, interpolated);
    previous = interpolated;
  }
  assert.ok(
    traversed <= trueAngle + SO3_GEODESIC_TOLERANCE_RAD,
    `path length ${deg(traversed)} deg exceeds true ${deg(trueAngle)} deg`,
  );
});

test("a 2*pi representation gap between adjacent samples renders no torch spin", () => {
  // The accepted plan legally carries a whole-turn RPY jump between adjacent
  // samples: the planner emits principal-range angles from a branch-crossing
  // extraction, so two consecutive samples can denote the SAME rotation while
  // their yaw components differ by 2*pi. Componentwise lerp renders that as a
  // full-turn spin the robot never performs.
  const from: Rpy3 = [rad(12), rad(-30), rad(170)];
  const to: Rpy3 = [rad(12), rad(-30), rad(170) + TWO_PI];
  assert.ok(
    so3RpyGeodesicAngleRad(from, to) <= SO3_GEODESIC_TOLERANCE_RAD,
    "the 2*pi-gapped pair must denote the same rotation",
  );

  let worstFromStart = 0;
  for (let sample = 1; sample <= 512; sample += 1) {
    worstFromStart = Math.max(
      worstFromStart,
      so3RpyGeodesicAngleRad(from, so3InterpolateRpy(from, to, sample / 512)),
    );
  }
  assert.ok(
    worstFromStart <= SO3_GEODESIC_TOLERANCE_RAD,
    `a 2*pi representation gap swept ${deg(worstFromStart)} deg of visible ` +
      "rotation; the segment denotes 0 deg",
  );
});

test("whole-turn shifted (non-principal) inputs interpolate the same rotation path", () => {
  const baseFrom: Rpy3 = [rad(200), rad(40), rad(-350)];
  const baseTo: Rpy3 = [rad(-150), rad(-30), rad(300)];
  const turnOffsets = [-2, 0, 3];
  let worst = 0;
  for (const fromTurns of turnOffsets) {
    for (const toTurns of turnOffsets) {
      const shiftedFrom: Rpy3 = [
        baseFrom[0] + fromTurns * TWO_PI,
        baseFrom[1] + fromTurns * TWO_PI,
        baseFrom[2] + fromTurns * TWO_PI,
      ];
      const shiftedTo: Rpy3 = [
        baseTo[0] + toTurns * TWO_PI,
        baseTo[1] + toTurns * TWO_PI,
        baseTo[2] + toTurns * TWO_PI,
      ];
      assert.ok(
        so3RpyGeodesicAngleRad(baseFrom, shiftedFrom) <= SO3_GEODESIC_TOLERANCE_RAD,
        "a whole-turn shift moved the frame",
      );
      for (let sample = 1; sample <= 256; sample += 1) {
        const alpha = sample / 256;
        worst = Math.max(
          worst,
          so3RpyGeodesicAngleRad(
            so3InterpolateRpy(baseFrom, baseTo, alpha),
            so3InterpolateRpy(shiftedFrom, shiftedTo, alpha),
          ),
        );
      }
    }
  }
  assert.ok(
    worst <= SO3_GEODESIC_TOLERANCE_RAD,
    `whole-turn shifts changed the path by ${worst} rad`,
  );
});

test("rotation advances uniformly along a non-principal segment", () => {
  // What must be continuous is the ROTATION, not its RPY spelling: a principal
  // extraction crosses branch cuts, so adjacent components may jump a whole
  // turn. Deliberately not asserted here, exactly as in the C++ suite.
  const baseFrom: Rpy3 = [rad(200), rad(40), rad(-350)];
  const baseTo: Rpy3 = [rad(-150), rad(-30), rad(300)];
  const samples = 256;
  const expectedStep = so3RpyGeodesicAngleRad(baseFrom, baseTo) / samples;
  let previous: Rpy3 = baseFrom;
  let worstStep = 0;
  for (let sample = 1; sample <= samples; sample += 1) {
    const interpolated = so3InterpolateRpy(baseFrom, baseTo, sample / samples);
    worstStep = Math.max(worstStep, so3RpyGeodesicAngleRad(previous, interpolated));
    previous = interpolated;
  }
  assert.ok(
    worstStep <= expectedStep + SO3_GEODESIC_TOLERANCE_RAD,
    `worst adjacent rotation step ${worstStep} rad exceeds the uniform ` +
      `geodesic step ${expectedStep} rad`,
  );
});

test("a constant-orientation segment keeps its frame bit-identically", () => {
  const shared: Rpy3 = [rad(-173.1051339449), rad(44.1360294638), rad(369.8510761166)];
  let mismatches = 0;
  for (let sample = 1; sample <= 512; sample += 1) {
    const interpolated = so3InterpolateRpy(shared, shared, sample / 512);
    for (let axis = 0; axis < 3; axis += 1) {
      if (interpolated[axis] !== shared[axis]) mismatches += 1;
    }
  }
  assert.equal(mismatches, 0, `${mismatches} of 1536 components drifted`);
});

test("pure translation stays a bit-identical componentwise lerp", () => {
  const from = [0.62, -0.19, 0.76, rad(-173.1051339449), rad(44.1360294638), rad(369.851)];
  const to = [0.95, 0.19, 0.61, from[3]!, from[4]!, from[5]!];
  for (let sample = 1; sample <= 512; sample += 1) {
    const alpha = sample / 512;
    const pose = interpolateTcpPose(from, to, alpha);
    for (let axis = 0; axis < 3; axis += 1) {
      assert.equal(
        pose[axis],
        from[axis]! + (to[axis]! - from[axis]!) * alpha,
        `translation axis ${axis} is not the planner's expression at alpha=${alpha}`,
      );
    }
    for (let axis = 3; axis < 6; axis += 1) {
      assert.equal(
        pose[axis],
        from[axis],
        `orientation axis ${axis} acquired rotation noise at alpha=${alpha}`,
      );
    }
  }
});

test("endpoints are returned verbatim", () => {
  const atZero = so3InterpolateRpy(PAIR24_FROM, PAIR24_TO, 0);
  const atOne = so3InterpolateRpy(PAIR24_FROM, PAIR24_TO, 1);
  for (let axis = 0; axis < 3; axis += 1) {
    assert.equal(atZero[axis], PAIR24_FROM[axis], `alpha=0 axis ${axis} not bit-exact`);
    assert.equal(atOne[axis], PAIR24_TO[axis], `alpha=1 axis ${axis} not bit-exact`);
  }
  const poseZero = interpolateTcpPose([1, 2, 3, ...PAIR24_FROM], [4, 5, 6, ...PAIR24_TO], 0);
  const poseOne = interpolateTcpPose([1, 2, 3, ...PAIR24_FROM], [4, 5, 6, ...PAIR24_TO], 1);
  assert.deepEqual(poseZero, [1, 2, 3, ...PAIR24_FROM]);
  assert.deepEqual(poseOne, [4, 5, 6, ...PAIR24_TO]);
});

test("slerp is unit norm, antipodal-sign invariant, finite and deterministic", () => {
  const from: Rpy3 = [rad(10), rad(20), rad(30)];
  const to: Rpy3 = [rad(-170), rad(-20), rad(-150)];
  const a = so3QuaternionFromRpy(from);
  const bPositive = so3QuaternionFromRpy(to);
  const bNegated: So3Quaternion = {
    w: -bPositive.w,
    x: -bPositive.x,
    y: -bPositive.y,
    z: -bPositive.z,
  };

  let worstNormError = 0;
  let worstSignDelta = 0;
  for (let sample = 0; sample <= 100; sample += 1) {
    const alpha = sample / 100;
    const q = so3QuaternionSlerp(a, bNegated, alpha);
    worstNormError = Math.max(
      worstNormError,
      Math.abs(Math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z) - 1),
    );
    worstSignDelta = Math.max(
      worstSignDelta,
      so3RotationGeodesicAngleRad(
        so3QuaternionRotationMatrix(q),
        so3QuaternionRotationMatrix(so3QuaternionSlerp(a, bPositive, alpha)),
      ),
    );
  }
  assert.ok(worstNormError <= 1.0e-12, `worst |norm-1| ${worstNormError}`);
  assert.ok(
    worstSignDelta <= SO3_GEODESIC_TOLERANCE_RAD,
    `sign flip changed the path by ${worstSignDelta} rad`,
  );

  const degenerate = so3InterpolateRpy(from, from, 0.5);
  assert.ok(degenerate.every(Number.isFinite), "degenerate segment produced non-finite RPY");

  for (let repeat = 0; repeat < 64; repeat += 1) {
    assert.deepEqual(
      so3InterpolateRpy(PAIR24_FROM, PAIR24_TO, 0.37),
      so3InterpolateRpy(PAIR24_FROM, PAIR24_TO, 0.37),
    );
  }
});

test("whole-turn unwrap is exact and rotation preserving", () => {
  assert.equal(so3UnwrapAngleToward(0.25, 0.25), 0.25);
  assert.ok(Math.abs(so3UnwrapAngleToward(0.1, 6.4) - (0.1 + TWO_PI)) < 1.0e-12);
});

// ---------------------------------------------------------------------------
// Cross-implementation pin.
//
// The tables below are the ACTUAL output of the reference implementation,
// motion-server/v1/src/robot_v4_cartesian_so3.hpp so3_interpolate_rpy(),
// dumped at 17 significant digits by compiling that header unmodified
// (c++ -std=c++17 -O2 -I src). This pins the TypeScript port to the numbers the
// Motion Server actually emits — not merely to the same tolerance band — so the
// two implementations cannot drift apart silently.
//
// Both the RPY components AND the rotation they denote are checked: the
// componentwise bound pins the representation (including the whole-turn
// branch-flip choices visible in the antipodal case), the geodesic bound pins
// the geometry. The componentwise bound is 1e-9 rad rather than exact equality
// because V8's Math and glibc's libm may differ by an ulp in cos/sin/atan2.
// ---------------------------------------------------------------------------
const CPP_COMPONENT_TOLERANCE_RAD = 1.0e-9;

interface CppCase {
  name: string;
  from: Rpy3;
  to: Rpy3;
  expected: [number, Rpy3][];
}

const CPP_REFERENCE_CASES: CppCase[] = [
  {
    name: "pair-24 inter-seam transition (177.33 deg true reorientation)",
    from: [-3.0212545394448225, 0.77031903290096937, 6.455119020278552],
    to: [0, 0, 6.2831853071795862],
    expected: [
      [0.10000000000000001, [-2.6375444931151053, 0.71763533851353578, 6.5988385247477801]],
      [0.25, [-2.1253214418879178, 0.57821836088994816, 6.7346023669782991]],
      [0.37, [-1.7640176102966891, 0.44028727668568812, 6.7706580329542554]],
      [0.5, [-1.3996532123505248, 0.28861669048210831, 6.7480943291526643]],
      [0.501, [-1.3969050097760822, 0.28749279967227409, 6.7477035298008419]],
      [0.75, [-0.71139559637366168, 0.059807809438120946, 6.5669755824065943]],
      [0.90000000000000002, [-0.28629875001773003, -0.00072553663381899895, 6.4012500549887239]],
    ],
  },
  {
    name: "yaw +/-180 deg branch-cut wrap (2 deg true)",
    from: [0, 0, 3.12413936106985],
    to: [0, 0, -3.12413936106985],
    expected: [
      [0.10000000000000001, [0, 0, 3.1276300195738385]],
      [0.25, [0, 0, 3.1328660073298216]],
      [0.37, [0, 0, 3.1370547975346081]],
      [0.5, [0, 0, 3.1415926535897931]],
      [0.501, [0, 0, 3.1416275601748329]],
      [0.75, [0, 0, 3.1503192998497647]],
      [0.90000000000000002, [0, 0, 3.1555552876057478]],
    ],
  },
  {
    name: "2*pi adjacent-sample representation gap (0 deg true)",
    from: [0.20943951023931953, -0.52359877559829882, 2.9670597283903604],
    to: [0.20943951023931953, -0.52359877559829882, 9.2502450355699466],
    expected: [
      [0.10000000000000001, [0.2094395102393195, -0.52359877559829882, 2.9670597283903604]],
      [0.25, [0.20943951023931953, -0.52359877559829882, 2.9670597283903604]],
      [0.37, [0.2094395102393195, -0.52359877559829882, 2.9670597283903604]],
      [0.5, [0.2094395102393195, -0.52359877559829882, 2.9670597283903604]],
      [0.501, [0.2094395102393195, -0.52359877559829882, 2.9670597283903604]],
      [0.75, [0.2094395102393195, -0.52359877559829882, 2.9670597283903604]],
      [0.90000000000000002, [0.2094395102393195, -0.52359877559829882, 2.9670597283903604]],
    ],
  },
  {
    name: "non-principal whole-turn-shifted pair (|component| > 2*pi)",
    from: [22.340214425527417, 19.54768762233649, 12.740903539558605],
    to: [-15.184364492350667, -13.089969389957471, -7.3303828583761836],
    expected: [
      [0.10000000000000001, [22.27878199389896, 19.426844477334647, 12.596957089352623]],
      [0.25, [22.229791846388014, 19.236056281823053, 12.412911570863436]],
      [0.37, [22.218964072757579, 19.079824355455429, 12.2807769499831]],
      [0.5, [22.230770110433319, 18.910627558574149, 12.142872580726925]],
      [0.501, [22.2309506886644, 18.909334398075494, 12.141812030030421]],
      [0.75, [22.319934562551367, 18.597429324958039, 11.86399786523703]],
      [0.90000000000000002, [22.421820875154687, 18.42695282085748, 11.668891038848621]],
    ],
  },
  {
    name: "antipodal-quaternion pair (shortest-arc sign fold)",
    from: [0.17453292519943295, 0.3490658503988659, 0.52359877559829882],
    to: [-2.9670597283903604, -0.3490658503988659, -2.6179938779914944],
    expected: [
      [0.10000000000000001, [0.174532925199433, 0.034906585039886542, 0.52359877559829893]],
      [0.25, [0.17453292519943292, -0.43633231299858244, 0.52359877559829893]],
      [0.37, [0.17453292519943289, -0.81332343142935792, 0.52359877559829893]],
      [0.5, [0.17453292519943289, -1.2217304763960308, 0.52359877559829859]],
      [0.501, [0.17453292519943317, -1.2248720690496209, 0.52359877559829915]],
      [0.75, [3.3161255787892259, -1.1344640137963147, 3.6651914291880918]],
      [0.90000000000000002, [3.3161255787892259, -0.66322511575784537, 3.6651914291880918]],
    ],
  },
];

for (const testCase of CPP_REFERENCE_CASES) {
  test(`matches the C++ reference implementation: ${testCase.name}`, () => {
    for (const [alpha, expected] of testCase.expected) {
      const actual = so3InterpolateRpy(testCase.from, testCase.to, alpha);
      for (let axis = 0; axis < 3; axis += 1) {
        const delta = Math.abs(actual[axis]! - expected[axis]!);
        assert.ok(
          delta <= CPP_COMPONENT_TOLERANCE_RAD,
          `alpha=${alpha} axis ${axis}: got ${actual[axis]}, C++ emits ` +
            `${expected[axis]} (delta ${delta} rad)`,
        );
      }
      const geodesic = so3RpyGeodesicAngleRad(actual, expected);
      assert.ok(
        geodesic <= SO3_GEODESIC_TOLERANCE_RAD,
        `alpha=${alpha}: the frame differs from the C++ frame by ${geodesic} rad`,
      );
    }
  });
}
