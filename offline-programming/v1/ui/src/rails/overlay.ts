/**
 * The rail view, ported from the weld planner's viewer: the rail as one
 * continuous line offset from the seam along the reference direction (the
 * datum work angle 0 is measured from), and the comb — one hair per sample
 * stride showing where the torch body sits at the chosen work and travel
 * angles. The cross-section inset did NOT come along; the angle popup is
 * where the numbers live.
 *
 * Geometry is built in the part's local frame (the weld planner's frames are
 * in STEP coordinates, metres); the caller poses the returned group with the
 * workpiece placement, the same rigid transform the workpiece mesh gets.
 */

import * as THREE from "three";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";

import type { RailsFrames } from "./client";
import { applyAngles, travelTangent } from "./frame-math";
import type { V3 } from "./frame-math";
import { containsS, fractionAt } from "./span-math";
import type { Span } from "./span-math";

const RAIL_COLOR = 0x2a6238;
/** The comb's pins and the seam guides: a light amber. The palette's
 *  caution ink (0x7f4200) reads as brown on the grey field at hairline
 *  width. */
export const RAILS_AMBER = 0xe08a2e;
const COMB_COLOR = RAILS_AMBER;
/** The moving direction arrows: the accent blue. */
const FLOW_COLOR = 0x1a45c4;
/** The rail's width in CSS pixels: 1.5x the 1 px a plain line draws.
 *  WebGL ignores `linewidth` on plain lines, so the rail is a fat line. */
const RAIL_WIDTH_PX = 1.5;
const RAIL_OFFSET_M = 0.02;
const COMB_HAIRS = 24;
/** Moving direction arrows: how many ride each weld, and how fast, in real
 *  millimetres — the motion IS the direction readout. */
const FLOW_CONES = 4;
const FLOW_SPEED_M_S = 0.04;
const FLOW_CONE_LENGTH_M = 0.009;
const FLOW_CONE_RADIUS_M = 0.0028;

interface FlowData {
  /** The seam polyline in draw order, with cumulative arc length. */
  points: V3[];
  cumulative: number[];
  total: number;
  /** Travel runs the polyline backwards. */
  reverse: boolean;
  cones: THREE.Mesh[];
}

const FLOW = new WeakMap<THREE.Group, FlowData>();
const CONE_GEOMETRY = new THREE.ConeGeometry(FLOW_CONE_RADIUS_M, FLOW_CONE_LENGTH_M, 8);
// The cone's own axis is +Y; rotate the geometry so +Z is "forward" and a
// lookAt-style quaternion can aim it along the travel tangent.
CONE_GEOMETRY.rotateX(Math.PI / 2);

export interface AnglePath {
  knotFractions: number[];
  workDeg: number[];
  travelDeg: number[];
  reverse: boolean;
}

/** Piecewise-linear read of the optimiser's path at a fraction. */
function pathAt(path: AnglePath, fraction: number): { workDeg: number; travelDeg: number } {
  const knots = path.knotFractions;
  if (!knots.length) return { workDeg: 0, travelDeg: 0 };
  if (fraction <= knots[0]!) return { workDeg: path.workDeg[0]!, travelDeg: path.travelDeg[0]! };
  const last = knots.length - 1;
  if (fraction >= knots[last]!) {
    return { workDeg: path.workDeg[last]!, travelDeg: path.travelDeg[last]! };
  }
  for (let i = 1; i < knots.length; i += 1) {
    if (fraction <= knots[i]!) {
      const t = (fraction - knots[i - 1]!) / (knots[i]! - knots[i - 1]! || 1);
      return {
        workDeg: path.workDeg[i - 1]! + t * (path.workDeg[i]! - path.workDeg[i - 1]!),
        travelDeg: path.travelDeg[i - 1]! + t * (path.travelDeg[i]! - path.travelDeg[i - 1]!),
      };
    }
  }
  return { workDeg: path.workDeg[last]!, travelDeg: path.travelDeg[last]! };
}

/**
 * Rail + comb for one weld, in part-local coordinates.
 *
 * With `angles` absent only the rail is drawn — a candidate nobody has
 * chosen angles for must not show a comb of decisions that were not made
 * (the weld planner's own rule).
 */
export function buildRailsOverlay(
  frames: RailsFrames,
  span: Span | null,
  angles: AnglePath | null,
): THREE.Group {
  const group = new THREE.Group();
  group.name = "rails-overlay-weld";
  let shown = frames.frames;
  if (span) {
    const inside = frames.frames.filter((f) => containsS(span, f.s));
    const ordered = span.end_s < span.start_s
      ? [...inside.filter((f) => f.s >= span.start_s), ...inside.filter((f) => f.s < span.start_s)]
      : inside;
    if (ordered.length >= 2) shown = ordered;
  }
  if (shown.length < 2) return group;

  const rail: number[] = [];
  for (const f of shown) {
    rail.push(
      f.point_xyz_m[0]! + f.reference_xyz[0]! * RAIL_OFFSET_M,
      f.point_xyz_m[1]! + f.reference_xyz[1]! * RAIL_OFFSET_M,
      f.point_xyz_m[2]! + f.reference_xyz[2]! * RAIL_OFFSET_M,
    );
  }
  const railMaterial = new LineMaterial({ color: RAIL_COLOR, linewidth: RAIL_WIDTH_PX, worldUnits: false });
  const railLine = new Line2(new LineGeometry().setPositions(rail), railMaterial);
  railLine.computeLineDistances();
  // Fat lines measure their width against the target they are drawn into;
  // the orbit view and the camera panes each have their own, so read it at
  // draw time.
  const viewport = new THREE.Vector4();
  railLine.onBeforeRender = (renderer) => {
    renderer.getCurrentViewport(viewport);
    railMaterial.resolution.set(Math.max(1, viewport.z), Math.max(1, viewport.w));
    railMaterial.linewidth = RAIL_WIDTH_PX * renderer.getPixelRatio();
  };
  railLine.renderOrder = 3;
  group.add(railLine);

  if (angles) {
    // Direction arrows: a few cones marching along the seam in the travel
    // direction. `tickOverlayFlow` moves them; without ticking they still
    // stand as static arrowheads pointing the right way.
    const flowPoints: V3[] = shown.map((f) => [
      f.point_xyz_m[0]!,
      f.point_xyz_m[1]!,
      f.point_xyz_m[2]!,
    ]);
    const cumulative: number[] = [0];
    for (let i = 1; i < flowPoints.length; i += 1) {
      const a = flowPoints[i - 1]!;
      const b = flowPoints[i]!;
      cumulative.push(cumulative[i - 1]! + Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]));
    }
    const total = cumulative[cumulative.length - 1]!;
    if (total > FLOW_CONE_LENGTH_M / 2) {
      // Short welds still get their arrow: fewer of them, shrunk to fit,
      // never none — a 20 mm weld is exactly the one whose direction is
      // hardest to read off the geometry.
      const count = Math.max(1, Math.min(FLOW_CONES, Math.floor(total / 0.02)));
      const shrink = Math.max(0.35, Math.min(1, total / (FLOW_CONE_LENGTH_M * 4)));
      const material = new THREE.MeshBasicMaterial({ color: FLOW_COLOR });
      const cones: THREE.Mesh[] = [];
      for (let i = 0; i < count; i += 1) {
        const cone = new THREE.Mesh(CONE_GEOMETRY, material);
        cone.scale.setScalar(shrink);
        cone.renderOrder = 4;
        group.add(cone);
        cones.push(cone);
      }
      const flow: FlowData = {
        points: flowPoints,
        cumulative,
        total,
        reverse: angles.reverse,
        cones,
      };
      FLOW.set(group, flow);
      tickOverlayFlow(group, 0);
    }

    const hairLength = Math.max(0.012, RAIL_OFFSET_M * 0.9);
    const stride = Math.max(1, Math.floor(shown.length / COMB_HAIRS));
    const hairs: number[] = [];
    for (let i = 0; i < shown.length; i += 1) {
      if (i % stride !== 0 && i !== shown.length - 1) continue;
      const f = shown[i]!;
      const fraction = span ? fractionAt(span, f.s) : f.s;
      const { workDeg, travelDeg } = pathAt(angles, fraction);
      const direction = applyAngles(
        travelTangent(f.tangent_xyz, angles.reverse),
        f.reference_xyz as V3,
        workDeg,
        travelDeg,
      );
      const p = f.point_xyz_m;
      const tip = [
        p[0]! + direction[0] * hairLength,
        p[1]! + direction[1] * hairLength,
        p[2]! + direction[2] * hairLength,
      ];
      if (!tip.every(Number.isFinite)) continue;
      hairs.push(p[0]!, p[1]!, p[2]!, tip[0]!, tip[1]!, tip[2]!);
    }
    const combGeometry = new THREE.BufferGeometry();
    combGeometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(hairs), 3));
    const comb = new THREE.LineSegments(
      combGeometry,
      new THREE.LineBasicMaterial({ color: COMB_COLOR, transparent: true, opacity: 0.85 }),
    );
    comb.renderOrder = 3;
    group.add(comb);
  }
  return group;
}

/** Advance the direction arrows to `timeS` — phase from absolute time, so
 *  the caller keeps no per-overlay clock. */
export function tickOverlayFlow(group: THREE.Group, timeS: number): void {
  const flow = FLOW.get(group);
  if (!flow) return;
  const at = (arc: number): { p: V3; t: V3 } => {
    const clamped = Math.min(Math.max(arc, 0), flow.total);
    let index = 1;
    while (index < flow.cumulative.length - 1 && flow.cumulative[index]! < clamped) index += 1;
    const a = flow.points[index - 1]!;
    const b = flow.points[index]!;
    const segment = flow.cumulative[index]! - flow.cumulative[index - 1]! || 1;
    const along = (clamped - flow.cumulative[index - 1]!) / segment;
    const p: V3 = [
      a[0] + (b[0] - a[0]) * along,
      a[1] + (b[1] - a[1]) * along,
      a[2] + (b[2] - a[2]) * along,
    ];
    const t: V3 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    return { p, t };
  };
  const travelled = (timeS * FLOW_SPEED_M_S) % flow.total;
  group.updateWorldMatrix(true, false);
  for (let i = 0; i < flow.cones.length; i += 1) {
    const spacing = flow.total / flow.cones.length;
    const forward = (travelled + i * spacing) % flow.total;
    const arc = flow.reverse ? flow.total - forward : forward;
    const { p, t } = at(arc);
    const cone = flow.cones[i]!;
    cone.position.set(p[0], p[1], p[2]);
    const direction = new THREE.Vector3(
      flow.reverse ? -t[0] : t[0],
      flow.reverse ? -t[1] : t[1],
      flow.reverse ? -t[2] : t[2],
    );
    if (direction.lengthSq() > 0) {
      // lookAt wants a WORLD-space target; positions here are group-local,
      // and the group carries the workpiece placement rotation.
      const target = group.localToWorld(
        new THREE.Vector3().copy(cone.position).add(direction),
      );
      cone.lookAt(target);
    }
  }
}

export function disposeOverlay(group: THREE.Group): void {
  group.traverse((node) => {
    const line = node as THREE.Line;
    if (line.geometry) line.geometry.dispose();
    const material = line.material as THREE.Material | undefined;
    if (material) material.dispose();
  });
  group.removeFromParent();
}
