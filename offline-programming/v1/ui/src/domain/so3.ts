// SO(3) orientation interpolation for browser-preview playback.
//
// TypeScript port of motion-server/v1/src/robot_v4_cartesian_so3.hpp.
// The Motion Server interpolates orientation on the SO(3) geodesic; the browser
// preview must render the SAME path, or the operator watches a torch spin the
// robot never performs. This module is the single implementation the preview
// uses — every consumer that interpolates a sampled orientation calls
// `interpolateTcpPose` / `so3InterpolateRpy` here rather than lerping RPY.
//
// Convention: every RPY triple is URDF fixed-axis roll/pitch/yaw, i.e.
//   R = Rz(yaw) * Ry(pitch) * Rx(roll)
// identical to the C++ header, to `spreadsheet_rpy_rotation_matrix()`, and to
// the Three.js "ZYX" Euler order that `applyTcpPose` (viewer/overlays.ts) uses
// to display these same components. Non-principal ("unwrapped") components with
// |angle| > pi are legal inputs: the authoring side emits them for seam
// continuity (weld.ts unwrapAngle), and a whole-turn shift leaves R exactly
// invariant.

export type Rpy3 = readonly [number, number, number];
export type Mat3 = [
  [number, number, number],
  [number, number, number],
  [number, number, number],
];

export interface So3Quaternion {
  w: number;
  x: number;
  y: number;
  z: number;
}

const TWO_PI = 6.283185307179586476925286766559;

/**
 * Deviation from the SO(3) geodesic that a sampled orientation path may carry.
 * Mirrors kSO3GeodesicToleranceRad in the C++ header: the componentwise Euler
 * lerp this module replaces reaches 0.370 rad (21.22 deg) on the measured Dog
 * Box pair-24 inter-seam transition.
 */
export const SO3_GEODESIC_TOLERANCE_RAD = 1.0e-6;

/** R = Rz(yaw) * Ry(pitch) * Rx(roll), with rpy = [roll, pitch, yaw]. */
export function so3RpyRotationMatrix(rpy: Rpy3): Mat3 {
  const cr = Math.cos(rpy[0]);
  const sr = Math.sin(rpy[0]);
  const cp = Math.cos(rpy[1]);
  const sp = Math.sin(rpy[1]);
  const cy = Math.cos(rpy[2]);
  const sy = Math.sin(rpy[2]);
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
}

/**
 * Inverse of `so3RpyRotationMatrix`, including the degenerate |cos(pitch)|
 * branch that pins yaw to zero. Returns principal-range angles.
 */
export function so3RotationMatrixRpy(rotation: Mat3): [number, number, number] {
  const pitch = Math.asin(clamp(-rotation[2][0], -1, 1));
  const cosPitch = Math.cos(pitch);
  if (Math.abs(cosPitch) > 1.0e-9) {
    return [
      Math.atan2(rotation[2][1], rotation[2][2]),
      pitch,
      Math.atan2(rotation[1][0], rotation[0][0]),
    ];
  }
  return [Math.atan2(-rotation[1][2], rotation[1][1]), pitch, 0];
}

export function so3QuaternionNormalize(q: So3Quaternion): So3Quaternion {
  const norm = Math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (!(norm > 0) || !Number.isFinite(norm)) {
    // Fail closed to identity rather than propagating NaN into the viewer.
    return { w: 1, x: 0, y: 0, z: 0 };
  }
  return { w: q.w / norm, x: q.x / norm, y: q.y / norm, z: q.z / norm };
}

/** q = qz(yaw) * qy(pitch) * qx(roll), matching the matrix composition above. */
export function so3QuaternionFromRpy(rpy: Rpy3): So3Quaternion {
  const cr = Math.cos(rpy[0] * 0.5);
  const sr = Math.sin(rpy[0] * 0.5);
  const cp = Math.cos(rpy[1] * 0.5);
  const sp = Math.sin(rpy[1] * 0.5);
  const cy = Math.cos(rpy[2] * 0.5);
  const sy = Math.sin(rpy[2] * 0.5);
  return so3QuaternionNormalize({
    w: cr * cp * cy + sr * sp * sy,
    x: sr * cp * cy - cr * sp * sy,
    y: cr * sp * cy + sr * cp * sy,
    z: cr * cp * sy - sr * sp * cy,
  });
}

export function so3QuaternionRotationMatrix(raw: So3Quaternion): Mat3 {
  const q = so3QuaternionNormalize(raw);
  return [
    [
      1 - 2 * (q.y * q.y + q.z * q.z),
      2 * (q.x * q.y - q.w * q.z),
      2 * (q.x * q.z + q.w * q.y),
    ],
    [
      2 * (q.x * q.y + q.w * q.z),
      1 - 2 * (q.x * q.x + q.z * q.z),
      2 * (q.y * q.z - q.w * q.x),
    ],
    [
      2 * (q.x * q.z - q.w * q.y),
      2 * (q.y * q.z + q.w * q.x),
      1 - 2 * (q.x * q.x + q.y * q.y),
    ],
  ];
}

export function so3QuaternionRpy(q: So3Quaternion): [number, number, number] {
  return so3RotationMatrixRpy(so3QuaternionRotationMatrix(q));
}

/**
 * Geodesic (angular) distance between two rotations: acos((tr(A^T B) - 1) / 2).
 * Invariant to the RPY representation of either operand.
 */
export function so3RotationGeodesicAngleRad(a: Mat3, b: Mat3): number {
  let relativeTrace = 0;
  for (let row = 0; row < 3; row += 1) {
    for (let column = 0; column < 3; column += 1) {
      relativeTrace += a[row]![column]! * b[row]![column]!;
    }
  }
  return Math.acos(clamp((relativeTrace - 1) / 2, -1, 1));
}

export function so3RpyGeodesicAngleRad(a: Rpy3, b: Rpy3): number {
  return so3RotationGeodesicAngleRad(so3RpyRotationMatrix(a), so3RpyRotationMatrix(b));
}

/**
 * Shortest-arc spherical linear interpolation.
 *
 * q and -q denote the same rotation, so a negative dot product means the raw
 * pair would be interpolated the long way round the 4-sphere; negating one
 * operand selects the <= pi traversal. Near-parallel inputs make sin(theta)
 * vanish, so fall back to normalized linear interpolation instead of dividing
 * by ~0.
 */
export function so3QuaternionSlerp(
  from: So3Quaternion,
  to: So3Quaternion,
  alpha: number,
): So3Quaternion {
  const a = so3QuaternionNormalize(from);
  let b = so3QuaternionNormalize(to);
  let dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
  if (dot < 0) {
    b = { w: -b.w, x: -b.x, y: -b.y, z: -b.z };
    dot = -dot;
  }
  const parallelDot = 1 - 1.0e-12;
  if (dot >= parallelDot) {
    return so3QuaternionNormalize({
      w: a.w + (b.w - a.w) * alpha,
      x: a.x + (b.x - a.x) * alpha,
      y: a.y + (b.y - a.y) * alpha,
      z: a.z + (b.z - a.z) * alpha,
    });
  }
  const theta = Math.acos(clamp(dot, -1, 1));
  const sinTheta = Math.sin(theta);
  const weightFrom = Math.sin((1 - alpha) * theta) / sinTheta;
  const weightTo = Math.sin(alpha * theta) / sinTheta;
  return so3QuaternionNormalize({
    w: a.w * weightFrom + b.w * weightTo,
    x: a.x * weightFrom + b.x * weightTo,
    y: a.y * weightFrom + b.y * weightTo,
    z: a.z * weightFrom + b.z * weightTo,
  });
}

/**
 * Shift `angle` by whole turns to sit nearest `reference`. The same rule as
 * weld.ts unwrapAngle; a 2*pi shift leaves the rotation invariant, so this only
 * chooses a representation.
 */
export function so3UnwrapAngleToward(angle: number, reference: number): number {
  const turns = Math.round((reference - angle) / TWO_PI);
  return angle + turns * TWO_PI;
}

/**
 * Interpolate orientation between two sampled frames.
 *
 * Endpoints are returned verbatim so an authored row target stays the exact
 * frame the operator authored. Interior samples are re-expressed as
 * whole-turn-unwrapped RPY near `fromRpy`, keeping the emitted sequence
 * continuous for non-principal inputs without changing the rotation it denotes.
 */
export function so3InterpolateRpy(
  fromRpy: Rpy3,
  toRpy: Rpy3,
  alpha: number,
): [number, number, number] {
  if (!(alpha > 0)) return [fromRpy[0], fromRpy[1], fromRpy[2]];
  if (alpha >= 1) return [toRpy[0], toRpy[1], toRpy[2]];
  // Constant-orientation segments — pure translation, and any straight seam
  // welded at a fixed torch frame — keep their frame BIT-IDENTICALLY. The old
  // componentwise lerp returned `from + 0 * alpha`, i.e. exactly `from`; a
  // quaternion round trip would instead inject ~1e-16 rad of atan2 noise. This
  // is exact equality on purpose: a genuinely tiny rotation must still slerp.
  if (fromRpy[0] === toRpy[0] && fromRpy[1] === toRpy[1] && fromRpy[2] === toRpy[2]) {
    return [fromRpy[0], fromRpy[1], fromRpy[2]];
  }
  const interpolated = so3QuaternionSlerp(
    so3QuaternionFromRpy(fromRpy),
    so3QuaternionFromRpy(toRpy),
    alpha,
  );
  const principal = so3QuaternionRpy(interpolated);
  return [
    so3UnwrapAngleToward(principal[0], fromRpy[0]),
    so3UnwrapAngleToward(principal[1], fromRpy[1]),
    so3UnwrapAngleToward(principal[2], fromRpy[2]),
  ];
}

/**
 * Interpolate a full 6-vector TCP pose: translation stays a componentwise lerp
 * (bit-identical to the arithmetic the planner uses for axes 0..2), orientation
 * travels the SO(3) geodesic.
 */
export function interpolateTcpPose(
  from: readonly number[],
  to: readonly number[],
  alpha: number,
): number[] {
  // Clamped like the componentwise lerp this replaces, so out-of-range alpha
  // still pins to an endpoint frame rather than extrapolating off the segment.
  const t = clamp(Number.isFinite(alpha) ? alpha : 0, 0, 1);
  const translation = [0, 1, 2].map((axis) => {
    const a = from[axis] ?? to[axis] ?? 0;
    const b = to[axis] ?? a;
    return a + (b - a) * t;
  });
  return [
    ...translation,
    ...so3InterpolateRpy(rpyOf(from, to), rpyOf(to, from), t),
  ];
}

function rpyOf(primary: readonly number[], fallback: readonly number[]): Rpy3 {
  return [
    primary[3] ?? fallback[3] ?? 0,
    primary[4] ?? fallback[4] ?? 0,
    primary[5] ?? fallback[5] ?? 0,
  ];
}

function clamp(value: number, minimum: number, maximum: number): number {
  return Math.max(minimum, Math.min(maximum, value));
}
