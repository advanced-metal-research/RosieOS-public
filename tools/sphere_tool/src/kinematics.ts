/**
 * Where every link of a cell is, for a given set of joint values.
 *
 * The twin of `python/weld_programmer/kinematics.py`, pinned to it by
 * `tests/programmer/test_kinematics_parity.py`. Duplicated on purpose and for the same
 * reason `frame-math.ts` is: dragging a positioner slider must move the cell
 * without a round trip, and the server owns the answer that reaches a
 * document.
 *
 * Forward kinematics only. No solver, no IK, no reachability -- those belong
 * to a planner, and GOAL.md draws the line at the weld program. What placement
 * needs is narrower and unavoidable: **the table moves**, so a workpiece
 * fixtured to it sits somewhere different for every value of J7/J8/J9.
 *
 * The convention, which must stay identical to the Python:
 *
 *     T_parent_child = Translate(xyz) . Rpy(rpy) . AxisAngle(axis, q)
 *     R              = Rz(yaw) . Ry(pitch) . Rx(roll)
 *
 * Row-major 4x4, sixteen numbers, the same layout `pose_document` emits. NOT
 * `THREE.Matrix4`, which is column-major -- `Matrix4.fromArray` reads
 * column-major and would transpose these silently, giving a cell that looks
 * assembled and is inside out. `toThreeArray` does the transpose in one named
 * place.
 */

/** Row-major 4x4. */
export type Mat4 = number[];
export type Vec3 = [number, number, number];

export interface CellJoint {
  name: string;
  type: string;
  parent: string;
  child: string;
  xyz_m: number[];
  rpy_rad: number[];
  axis_xyz: number[];
  limits_rad?: { lower: number; upper: number } | null;
}

export interface CellLink {
  name: string;
  visuals: { mesh: string; xyz_m: number[]; rpy_rad: number[]; rgba: number[] }[];
}

export interface Cell {
  id: string;
  frames: Record<string, string>;
  links: CellLink[];
  joints: CellJoint[];
  axes: string[];
  problems: string[];
}

export const identity = (): Mat4 => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

export function multiply(a: Mat4, b: Mat4): Mat4 {
  const out = new Array<number>(16).fill(0);
  for (let row = 0; row < 4; row += 1) {
    for (let col = 0; col < 4; col += 1) {
      let sum = 0;
      for (let k = 0; k < 4; k += 1) sum += a[row * 4 + k] * b[k * 4 + col];
      out[row * 4 + col] = sum;
    }
  }
  return out;
}

/**
 * URDF fixed-axis roll-pitch-yaw, `Rz(yaw) . Ry(pitch) . Rx(roll)`.
 *
 * Written out rather than composed from three products, matching the Python
 * exactly: this is the one function whose ORDER matters, and a product form
 * invites someone to tidy the operands.
 */
export function rpyMatrix(roll: number, pitch: number, yaw: number): Mat4 {
  const cr = Math.cos(roll);
  const sr = Math.sin(roll);
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  return [
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0,
    -sp, cp * sr, cp * cr, 0,
    0, 0, 0, 1,
  ];
}

/** Rodrigues rotation. A zero-length axis is an identity, never a NaN: a NaN
 *  transform puts a link at no position at all, and three.js culls that
 *  silently, so the limb would simply not be there. */
export function axisAngleMatrix(axis: number[], angle: number): Mat4 {
  const length = Math.hypot(axis[0], axis[1], axis[2]);
  if (length < 1e-12) return identity();
  const [x, y, z] = [axis[0] / length, axis[1] / length, axis[2] / length];
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const t = 1 - c;
  return [
    t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0,
    t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0,
    t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0,
    0, 0, 0, 1,
  ];
}

export function transform(xyz: number[], rpy: number[]): Mat4 {
  const out = rpyMatrix(rpy[0], rpy[1], rpy[2]);
  out[3] = xyz[0];
  out[7] = xyz[1];
  out[11] = xyz[2];
  return out;
}

/** Parent to child, the joint value applied AFTER the fixed origin. Before it
 *  -- rotating the parent then offsetting -- is the other plausible reading of
 *  a URDF and swings the limb about the wrong pivot. */
export function jointTransform(joint: CellJoint, value: number): Mat4 {
  const fixed = transform(joint.xyz_m, joint.rpy_rad);
  if (joint.type === "fixed") return fixed;
  return multiply(fixed, axisAngleMatrix(joint.axis_xyz, value));
}

/** Joint values held inside their declared limits. Clamped rather than
 *  refused: a slider dragged past a limit is the limit doing its job, and the
 *  viewport should show the pose the robot would actually hold. */
export function clampToLimits(cell: Cell, values: Record<string, number>): Record<string, number> {
  const out: Record<string, number> = {};
  for (const joint of cell.joints) {
    if (joint.type === "fixed") continue;
    const raw = values[joint.name];
    if (raw === undefined || !Number.isFinite(raw)) continue;
    out[joint.name] = joint.limits_rad
      ? Math.min(Math.max(raw, joint.limits_rad.lower), joint.limits_rad.upper)
      : raw;
  }
  return out;
}

function baseOf(cell: Cell): string | null {
  const names = cell.links.map((l) => l.name);
  const declared = cell.frames?.base;
  if (declared && names.includes(declared)) return declared;
  const parents = new Map(cell.joints.map((j) => [j.child, j.parent]));
  const rootOf = (name: string) => {
    const seen = new Set<string>();
    while (parents.has(name) && !seen.has(name)) {
      seen.add(name);
      name = parents.get(name)!;
    }
    return name;
  };
  const roots = names.filter((n) => !parents.has(n)).sort();
  let best: string | null = null;
  let bestCount = -1;
  for (const root of roots) {
    const count = names.filter((n) => rootOf(n) === root).length;
    if (count > bestCount) {
      bestCount = count;
      best = root;
    }
  }
  return best;
}

/**
 * A world transform for every link, keyed by link name.
 *
 * Walks parents rather than recursing over children, so joints listed in any
 * order resolve -- URDF does not promise topological order, and
 * `rosie_1400_v3` really does declare J9 after the arm's tool.
 *
 * A link that cannot reach the base is LEFT OUT, not placed at the origin. In
 * the middle of a cell the origin is a plausible-looking spot, so a limb
 * dropped there reads as geometry rather than as a missing joint.
 */
export function linkTransforms(cell: Cell, values: Record<string, number> = {}): Map<string, Mat4> {
  const byChild = new Map(cell.joints.map((j) => [j.child, j]));
  const posed = clampToLimits(cell, values);
  const base = baseOf(cell);
  const out = new Map<string, Mat4>();

  const resolve = (name: string, seen: Set<string>): Mat4 | null => {
    const cached = out.get(name);
    if (cached) return cached;
    const joint = byChild.get(name);
    if (!joint) return name === base ? identity() : null;
    if (seen.has(name)) throw new Error(`the joint chain loops at link ${name}`);
    const parent = resolve(joint.parent, new Set(seen).add(name));
    if (!parent) return null;
    const world = multiply(parent, jointTransform(joint, posed[joint.name] ?? 0));
    out.set(name, world);
    return world;
  };

  for (const link of cell.links) {
    const placed = resolve(link.name, new Set());
    if (placed) out.set(link.name, placed);
  }
  return out;
}

/** The world transform of a NAMED frame -- `base`, `tool` or `work`. By role,
 *  never by link name: `positioner_table_a` is a fact about one CAD export. */
export function frameOf(
  cell: Cell,
  role: string,
  values: Record<string, number> = {},
): Mat4 | null {
  const link = cell.frames?.[role];
  if (!link) return null;
  return linkTransforms(cell, values).get(link) ?? null;
}

/**
 * The inverse of `rpyMatrix`: fixed-axis roll-pitch-yaw from a rotation.
 *
 * At pitch = +/-90 degrees the first and third rotations act about the same
 * axis and only their SUM is recoverable -- classic gimbal lock. Roll is pinned
 * to zero there and the whole turn attributed to yaw: one of the infinitely
 * many correct answers, stable under small changes, and it round-trips
 * exactly, which is what a drag handle needs.
 */
export function rpyFromMatrix(m: Mat4): Vec3 {
  const sinPitch = Math.min(1, Math.max(-1, -m[8]));
  const pitch = Math.asin(sinPitch);
  if (Math.abs(sinPitch) > 1 - 1e-9) {
    return [0, pitch, Math.atan2(-m[1], m[5])];
  }
  return [Math.atan2(m[9], m[10]), pitch, Math.atan2(m[4], m[0])];
}

/** The inverse of a RIGID transform: `[R|t]` -> `[R^T | -R^T t]`. Not a general
 *  inverse -- every transform here is a rotation and a translation, so the
 *  transpose is exact and free where a general inversion would introduce error
 *  and hide a scale nobody meant to have. */
export function invert(m: Mat4): Mat4 {
  const t = [m[3], m[7], m[11]];
  return [
    m[0], m[4], m[8], -(m[0] * t[0] + m[4] * t[1] + m[8] * t[2]),
    m[1], m[5], m[9], -(m[1] * t[0] + m[5] * t[1] + m[9] * t[2]),
    m[2], m[6], m[10], -(m[2] * t[0] + m[6] * t[1] + m[10] * t[2]),
    0, 0, 0, 1,
  ];
}

/** Column-major from three.js back to the row-major used here. The exact
 *  inverse of `toThreeArray`, and the boundary a gizmo's matrix crosses. */
export function fromThreeArray(a: number[] | Float32Array): Mat4 {
  return [
    a[0], a[4], a[8], a[12],
    a[1], a[5], a[9], a[13],
    a[2], a[6], a[10], a[14],
    a[3], a[7], a[11], a[15],
  ];
}

export function applyMat(matrix: Mat4, point: Vec3): Vec3 {
  return [
    matrix[0] * point[0] + matrix[1] * point[1] + matrix[2] * point[2] + matrix[3],
    matrix[4] * point[0] + matrix[5] * point[1] + matrix[6] * point[2] + matrix[7],
    matrix[8] * point[0] + matrix[9] * point[1] + matrix[10] * point[2] + matrix[11],
  ];
}

/**
 * Row-major to the column-major array `THREE.Matrix4.fromArray` expects.
 *
 * The one place the transpose happens. Handing a row-major array straight to
 * three.js produces a cell that looks assembled and is inside out -- every
 * rotation transposed, which for a symmetric-ish robot is subtle enough to
 * argue with rather than obviously wrong.
 */
export function toThreeArray(matrix: Mat4): number[] {
  return [
    matrix[0], matrix[4], matrix[8], matrix[12],
    matrix[1], matrix[5], matrix[9], matrix[13],
    matrix[2], matrix[6], matrix[10], matrix[14],
    matrix[3], matrix[7], matrix[11], matrix[15],
  ];
}
