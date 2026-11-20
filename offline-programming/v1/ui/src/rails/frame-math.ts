/**
 * The two angle rotations, ported verbatim from the weld planner
 * (`weld_planner/v1/ui/src/frame-math.ts`). The server builds the frames so
 * no client can diverge on a sign convention; the client applies only these
 * rotations. Copied rather than imported because the two trees typecheck
 * under different tsconfigs — this is thirty lines of settled vector math,
 * and SEAM-MODEL.md section 5 is its specification.
 */

export type V3 = [number, number, number];

export const cross = (a: V3, b: V3): V3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];

const norm = (a: V3): V3 => {
  const n = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0] / n, a[1] / n, a[2] / n];
};

/** Rodrigues rotation of `v` about `axis` by `angle` radians, right-handed. */
export function rotate(axis: V3, angle: number, v: V3): V3 {
  const k = norm(axis);
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const kxv = cross(k, v);
  const kd = k[0] * v[0] + k[1] * v[1] + k[2] * v[2];
  return [
    v[0] * c + kxv[0] * s + k[0] * kd * (1 - c),
    v[1] * c + kxv[1] * s + k[1] * kd * (1 - c),
    v[2] * c + kxv[2] * s + k[2] * kd * (1 - c),
  ];
}

/**
 * The electrode direction for a frame `(t, r)` at the given work and travel
 * angles, per SEAM-MODEL.md: work rotates the reference about the tangent,
 * travel tilts about the rotated reference.
 */
export function applyAngles(t: V3, r: V3, workDeg: number, travelDeg: number): V3 {
  const rad = Math.PI / 180;
  const rPrime = rotate(t, workDeg * rad, r);
  const bPrime = cross(t, rPrime);
  return rotate(bPrime, -travelDeg * rad, rPrime);
}

/** The travel tangent, negated when the weld runs the seam the other way. */
export const travelTangent = (t: number[], reverse: boolean): V3 => {
  const v: V3 = [t[0] ?? 0, t[1] ?? 0, t[2] ?? 0];
  return reverse ? [-v[0], -v[1], -v[2]] : v;
};
