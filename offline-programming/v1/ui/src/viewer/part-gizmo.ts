import * as THREE from "three";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";
import { LineSegments2 } from "three/addons/lines/LineSegments2.js";
import { LineSegmentsGeometry } from "three/addons/lines/LineSegmentsGeometry.js";
import { disposeObjectTree } from "./resource-disposal.ts";

/**
 * The part-placement triad: one handle set that moves AND turns the part.
 *
 * three's TransformControls shows either arrows or rings, and the operator
 * has to switch modes between "slide it over" and "square it up". CAD
 * placement manipulators (Onshape, Fusion) put both on one gizmo: an arrow
 * per part axis, and a short rotation arc per axis -- a 45° piece of the
 * circle the part would turn on, with a knob at its middle -- sitting
 * between the other two arrows near their tips. This is that gizmo, in the
 * PART's axes -- the triad turns with the part, so an arrow always slides
 * it along one of its own edges and an arc always turns it about one of
 * its own axes -- at a constant screen size, drawn over everything so a
 * handle is never buried in the part it moves.
 *
 * Drags are read in the world: an arrow drag is the closest point on the
 * axis line to the pointer ray, an arc drag is the pointer's angle about
 * the pivot in the arc's plane, accumulated so a turn past 180° does not
 * snap back. The attached object's parent transform is respected on the
 * way back in.
 */

export type PartGizmoAxis = "x" | "y" | "z";
export type PartGizmoHandleKind = "translate" | "rotate";

export interface PartGizmoHandle {
  axis: PartGizmoAxis;
  kind: PartGizmoHandleKind;
}

export interface PartGizmoCallbacks {
  /** The operator grabbed (true) or released (false) a handle. */
  onDraggingChanged?: (dragging: boolean, handle: PartGizmoHandle) => void;
  /** The attached object moved under the pointer. */
  onObjectChange?: () => void;
  /** The pointer was released after a drag. */
  onMouseUp?: () => void;
}

export interface PartGizmoOptions {
  camera: THREE.PerspectiveCamera;
  domElement: HTMLElement;
  /** Gizmo length as a fraction of the camera distance (screen-constant). */
  size?: number;
  callbacks?: PartGizmoCallbacks;
}

/** One colour for every handle: the triad is in the PART's axes, so the
 *  usual red/green/blue would claim a match with the world axes that a
 *  turned part does not have. Hovered or held handle in yellow. */
export const PART_GIZMO_HANDLE_COLOR = 0x22262a;
export const PART_GIZMO_ACTIVE_COLOR = 0x1a45c4;
/** Line width in screen pixels. WebGL's own lines are 1 px and cannot be
 *  widened, so the handles are fat lines (three's Line2). */
export const PART_GIZMO_LINE_PX = 2;

/** Gizmo length as a fraction of the camera distance: at the orbit camera's
 *  46° vertical field the arrows span about a fifth of the pane, which is
 *  what TransformControls at size 0.8 gave. */
export const PART_GIZMO_DEFAULT_SIZE = 0.16;

/** Handle proportions, in gizmo units (the arrow is 1 long). */
const SHAFT_START = 0.14;
const CONE_LENGTH = 0.18;
const CONE_RADIUS = 0.055;
const PICK_RADIUS = 0.07;
/** The rotation arc is centred on the pivot in the plane the part turns in,
 *  out near the arrow tips, halfway between the other two arrows so it
 *  never sits on a shaft. A short arc reads as "turn", a full ring read as
 *  a third thing to hit; the knob at its middle is the grab point. */
const ARC_RADIUS = 0.78;
const ARC_SPAN_RAD = (45 * Math.PI) / 180;
const ARC_PICK_TUBE = 0.05;
const KNOB_RADIUS = 0.045;
const KNOB_PICK_RADIUS = 0.09;
const RENDER_ORDER = 1_000;

const AXIS_VECTORS: Readonly<Record<PartGizmoAxis, THREE.Vector3>> = {
  x: new THREE.Vector3(1, 0, 0),
  y: new THREE.Vector3(0, 1, 0),
  z: new THREE.Vector3(0, 0, 1),
};
/** The two arrows a rotation arc sits between, in right-hand order. */
const OTHER_AXES: Readonly<Record<PartGizmoAxis, [PartGizmoAxis, PartGizmoAxis]>> = {
  x: ["y", "z"],
  y: ["z", "x"],
  z: ["x", "y"],
};

export interface PartGizmoRay {
  origin: THREE.Vector3;
  direction: THREE.Vector3;
}

/**
 * Parameter along the line (point, unit axis) closest to the ray, or null
 * when the ray runs along the axis and every point is as close as the next.
 */
export function closestAxisParameter(
  ray: PartGizmoRay,
  point: THREE.Vector3,
  axis: THREE.Vector3,
): number | null {
  const w = ray.origin.clone().sub(point);
  const b = ray.direction.dot(axis);
  const denom = 1 - b * b;
  if (denom < 1e-6) return null;
  const u = (w.dot(axis) * b - w.dot(ray.direction)) / denom;
  return w.dot(axis) + u * b;
}

/**
 * Angle of the ray's hit on the plane (point, unit normal), measured in a
 * fixed basis of that plane, or null when the ray is parallel to the plane
 * (the arc is seen edge-on and the pointer says nothing about the turn).
 */
export function ringAngle(
  ray: PartGizmoRay,
  point: THREE.Vector3,
  normal: THREE.Vector3,
): number | null {
  const along = ray.direction.dot(normal);
  if (Math.abs(along) < 1e-4) return null;
  const t = point.clone().sub(ray.origin).dot(normal) / along;
  if (t < 0) return null;
  const hit = ray.origin.clone().addScaledVector(ray.direction, t).sub(point);
  const [e1, e2] = planeBasis(normal);
  return Math.atan2(hit.dot(e2), hit.dot(e1));
}

/** Shortest signed step from one ring angle to the next. */
export function wrapAngle(delta: number): number {
  let d = delta % (2 * Math.PI);
  if (d > Math.PI) d -= 2 * Math.PI;
  if (d < -Math.PI) d += 2 * Math.PI;
  return d;
}

function planeBasis(normal: THREE.Vector3): [THREE.Vector3, THREE.Vector3] {
  const helper = Math.abs(normal.x) < 0.9
    ? new THREE.Vector3(1, 0, 0)
    : new THREE.Vector3(0, 1, 0);
  const e1 = helper.clone().cross(normal).normalize();
  const e2 = normal.clone().cross(e1).normalize();
  return [e1, e2];
}

interface HandleRecord extends PartGizmoHandle {
  /** The drawn lines, sharing one material so a hover recolours them all. */
  visuals: Array<Line2 | LineSegments2>;
  material: LineMaterial;
  pick: THREE.Mesh;
  /** A second pick body (the arc under the knob). */
  pickExtra?: THREE.Mesh;
}

interface DragState {
  handle: HandleRecord;
  axisWorld: THREE.Vector3;
  /** World pose of the object when the handle was grabbed. */
  positionWorld: THREE.Vector3;
  quaternionWorld: THREE.Quaternion;
  /** Arrow: axis parameter under the pointer at grab. */
  startParameter: number;
  /** Arc: last pointer angle, and the turn accumulated since grab. */
  lastAngle: number;
  turnRad: number;
}

export class PartGizmo {
  /** Add to the scene; toggle `visible` to hide the whole triad. */
  readonly helper = new THREE.Group();
  private readonly camera: THREE.PerspectiveCamera;
  private readonly domElement: HTMLElement;
  private readonly size: number;
  private readonly callbacks: PartGizmoCallbacks;
  private readonly raycaster = new THREE.Raycaster();
  private readonly pointer = new THREE.Vector2();
  private readonly handles: HandleRecord[] = [];
  private attached: THREE.Object3D | null = null;
  private hovered: HandleRecord | null = null;
  private drag: DragState | null = null;
  private disposed = false;

  constructor(options: PartGizmoOptions) {
    this.camera = options.camera;
    this.domElement = options.domElement;
    this.size = options.size ?? PART_GIZMO_DEFAULT_SIZE;
    this.callbacks = options.callbacks ?? {};
    this.helper.name = "part-gizmo";
    this.helper.visible = false;
    for (const axis of ["x", "y", "z"] as const) {
      this.handles.push(this.buildArrow(axis), this.buildArc(axis));
    }
    // Drawn after the scene, with depth off: a handle inside the part is
    // still a handle.
    this.helper.traverse((node) => {
      node.renderOrder = RENDER_ORDER;
    });
    this.domElement.addEventListener("pointerdown", this.onPointerDown);
    this.domElement.addEventListener("pointermove", this.onPointerMove);
    this.domElement.addEventListener("pointerup", this.onPointerUp);
    this.domElement.addEventListener("pointercancel", this.onPointerUp);
    this.domElement.addEventListener("pointerleave", this.onPointerLeave);
  }

  get object(): THREE.Object3D | null {
    return this.attached;
  }

  get dragging(): boolean {
    return this.drag !== null;
  }

  attach(object: THREE.Object3D): void {
    this.attached = object;
    this.helper.visible = true;
    this.update();
  }

  detach(): void {
    if (this.drag) this.endDrag();
    this.attached = null;
    this.helper.visible = false;
    this.setHovered(null);
  }

  /**
   * Follow the attached object's pose and hold a constant screen size. Call
   * once per frame before rendering the pane the gizmo answers to. The
   * triad takes the object's rotation, so mid-turn the arrows and arcs are
   * seen turning with the part. `pixelRatio` is the renderer's: fat lines
   * measure their width against the drawing buffer, not CSS pixels.
   */
  update(pixelRatio = 1): void {
    const object = this.attached;
    if (!object) return;
    const rect = this.domElement.getBoundingClientRect();
    const ratio = pixelRatio > 0 ? pixelRatio : 1;
    const width = Math.max(1, rect.width * ratio);
    const height = Math.max(1, rect.height * ratio);
    for (const handle of this.handles) handle.material.resolution.set(width, height);
    object.updateWorldMatrix(true, false);
    object.getWorldPosition(this.helper.position);
    object.getWorldQuaternion(this.helper.quaternion);
    const distance = this.camera.position.distanceTo(this.helper.position);
    const scale = Math.max(distance * this.size, 1e-3);
    this.helper.scale.setScalar(scale);
  }

  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    this.detach();
    this.domElement.removeEventListener("pointerdown", this.onPointerDown);
    this.domElement.removeEventListener("pointermove", this.onPointerMove);
    this.domElement.removeEventListener("pointerup", this.onPointerUp);
    this.domElement.removeEventListener("pointercancel", this.onPointerUp);
    this.domElement.removeEventListener("pointerleave", this.onPointerLeave);
    this.domElement.style.cursor = "";
    this.helper.removeFromParent();
    disposeObjectTree(this.helper);
    this.helper.clear();
  }

  private buildArrow(axis: PartGizmoAxis): HandleRecord {
    const material = lineMaterial(PART_GIZMO_HANDLE_COLOR);
    // Outlines only: a shaft line and the eight edges of a square pyramid
    // head, which reads as an arrowhead from any angle.
    const shaft = new Line2(
      new LineGeometry().setPositions([0, SHAFT_START, 0, 0, 1 - CONE_LENGTH, 0]),
      material,
    );
    const cone = new LineSegments2(
      new LineSegmentsGeometry().fromEdgesGeometry(
        new THREE.EdgesGeometry(new THREE.ConeGeometry(CONE_RADIUS, CONE_LENGTH, 4)),
      ),
      material,
    );
    cone.position.y = 1 - CONE_LENGTH / 2;
    const pick = new THREE.Mesh(
      new THREE.CylinderGeometry(PICK_RADIUS, PICK_RADIUS, 1 - SHAFT_START, 8),
      pickMaterial(),
    );
    pick.position.y = SHAFT_START + (1 - SHAFT_START) / 2;
    const group = new THREE.Group();
    group.name = `part-gizmo-translate-${axis}`;
    group.add(shaft, cone, pick);
    orientAlong(group, axis);
    this.helper.add(group);
    const record: HandleRecord = { axis, kind: "translate", visuals: [shaft, cone], material, pick };
    pick.userData.handle = record;
    return record;
  }

  private buildArc(axis: PartGizmoAxis): HandleRecord {
    const material = lineMaterial(PART_GIZMO_HANDLE_COLOR);
    // Outlines only: the arc as a curve, the knob as a circle in its plane.
    const arc = new Line2(
      new LineGeometry().setPositions(
        flatPoints(new THREE.EllipseCurve(0, 0, ARC_RADIUS, ARC_RADIUS, 0, ARC_SPAN_RAD, false, 0).getPoints(32)),
      ),
      material,
    );
    const mid = new THREE.Vector3(
      ARC_RADIUS * Math.cos(ARC_SPAN_RAD / 2),
      ARC_RADIUS * Math.sin(ARC_SPAN_RAD / 2),
      0,
    );
    const circle = new THREE.EllipseCurve(0, 0, KNOB_RADIUS, KNOB_RADIUS, 0, 2 * Math.PI, false, 0).getPoints(24);
    circle.push(circle[0]!.clone());
    const knob = new Line2(new LineGeometry().setPositions(flatPoints(circle)), material);
    knob.position.copy(mid);
    const pickArc = new THREE.Mesh(
      new THREE.TorusGeometry(ARC_RADIUS, ARC_PICK_TUBE, 6, 16, ARC_SPAN_RAD),
      pickMaterial(),
    );
    const pick = new THREE.Mesh(new THREE.SphereGeometry(KNOB_PICK_RADIUS, 10, 8), pickMaterial());
    pick.position.copy(mid);
    const group = new THREE.Group();
    group.name = `part-gizmo-rotate-${axis}`;
    group.add(arc, knob, pickArc, pick);
    // The torus lies in XY with its arc running from local +X towards +Y.
    // Stand it up so its normal is the axis, then spin it about that axis
    // until the arc's middle points along the bisector of the other two
    // arrows.
    const normal = AXIS_VECTORS[axis];
    const stand = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), normal);
    const [a, b] = OTHER_AXES[axis];
    const bisector = AXIS_VECTORS[a].clone().add(AXIS_VECTORS[b]).normalize();
    const start = new THREE.Vector3(1, 0, 0).applyQuaternion(stand);
    const toBisector = Math.atan2(start.clone().cross(bisector).dot(normal), start.dot(bisector));
    const spin = new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(0, 0, 1),
      toBisector - ARC_SPAN_RAD / 2,
    );
    group.quaternion.copy(stand).multiply(spin);
    this.helper.add(group);
    const record: HandleRecord = { axis, kind: "rotate", visuals: [arc, knob], material, pick, pickExtra: pickArc };
    pick.userData.handle = record;
    pickArc.userData.handle = record;
    return record;
  }

  private pickHandle(event: PointerEvent): HandleRecord | null {
    if (!this.attached || !this.helper.visible) return null;
    this.setPointer(event);
    const bodies = this.handles.flatMap((handle) =>
      handle.pickExtra ? [handle.pick, handle.pickExtra] : [handle.pick]);
    const hits = this.raycaster.intersectObjects(bodies, false);
    const hit = hits[0];
    return hit ? (hit.object.userData.handle as HandleRecord) : null;
  }

  private setPointer(event: PointerEvent): void {
    const rect = this.domElement.getBoundingClientRect();
    this.pointer.set(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1,
    );
    this.raycaster.setFromCamera(this.pointer, this.camera);
  }

  private currentRay(): PartGizmoRay {
    return {
      origin: this.raycaster.ray.origin.clone(),
      direction: this.raycaster.ray.direction.clone().normalize(),
    };
  }

  private onPointerDown = (event: PointerEvent): void => {
    if (event.button !== 0 || this.drag || !this.attached) return;
    const handle = this.pickHandle(event);
    if (!handle) return;
    event.preventDefault();
    event.stopImmediatePropagation();
    const object = this.attached;
    object.updateWorldMatrix(true, false);
    const positionWorld = object.getWorldPosition(new THREE.Vector3());
    const quaternionWorld = object.getWorldQuaternion(new THREE.Quaternion());
    // The handle's axis is the part's, as it stands when grabbed. A turn is
    // about that axis, which the turn itself leaves where it is.
    const axisWorld = AXIS_VECTORS[handle.axis].clone().applyQuaternion(quaternionWorld).normalize();
    const ray = this.currentRay();
    const startParameter = closestAxisParameter(ray, positionWorld, axisWorld) ?? 0;
    const lastAngle = ringAngle(ray, positionWorld, axisWorld) ?? 0;
    this.drag = {
      handle,
      axisWorld,
      positionWorld,
      quaternionWorld,
      startParameter,
      lastAngle,
      turnRad: 0,
    };
    try {
      this.domElement.setPointerCapture(event.pointerId);
    } catch {
      // Not every pane element can capture (tests, detached nodes); the
      // move/up listeners on the pane still see the drag.
    }
    this.setHovered(handle);
    this.callbacks.onDraggingChanged?.(true, handle);
  };

  private onPointerMove = (event: PointerEvent): void => {
    if (!this.attached) return;
    const drag = this.drag;
    if (!drag) {
      const handle = this.pickHandle(event);
      this.setHovered(handle);
      return;
    }
    event.preventDefault();
    this.setPointer(event);
    const ray = this.currentRay();
    const object = this.attached;
    if (drag.handle.kind === "translate") {
      const parameter = closestAxisParameter(ray, drag.positionWorld, drag.axisWorld);
      if (parameter === null) return;
      const next = drag.positionWorld.clone()
        .addScaledVector(drag.axisWorld, parameter - drag.startParameter);
      setWorldPosition(object, next);
    } else {
      const angle = ringAngle(ray, drag.positionWorld, drag.axisWorld);
      if (angle === null) return;
      drag.turnRad += wrapAngle(angle - drag.lastAngle);
      drag.lastAngle = angle;
      const turn = new THREE.Quaternion().setFromAxisAngle(drag.axisWorld, drag.turnRad);
      setWorldQuaternion(object, turn.multiply(drag.quaternionWorld));
    }
    this.callbacks.onObjectChange?.();
  };

  private onPointerUp = (event: PointerEvent): void => {
    if (!this.drag) return;
    try {
      this.domElement.releasePointerCapture(event.pointerId);
    } catch {
      // Capture was never taken; nothing to release.
    }
    this.endDrag();
    this.callbacks.onMouseUp?.();
  };

  private onPointerLeave = (): void => {
    if (!this.drag) this.setHovered(null);
  };

  private endDrag(): void {
    const drag = this.drag;
    if (!drag) return;
    this.drag = null;
    this.callbacks.onDraggingChanged?.(false, drag.handle);
  }

  private setHovered(handle: HandleRecord | null): void {
    if (this.hovered === handle) return;
    this.hovered = handle;
    for (const record of this.handles) {
      const active = record === handle;
      const color = active ? PART_GIZMO_ACTIVE_COLOR : PART_GIZMO_HANDLE_COLOR;
      record.material.color.setHex(color);
    }
    this.domElement.style.cursor = handle
      ? (handle.kind === "rotate" ? "grab" : "move")
      : "";
  }
}

function lineMaterial(color: number): LineMaterial {
  return new LineMaterial({
    color,
    linewidth: PART_GIZMO_LINE_PX,
    worldUnits: false,
    transparent: true,
    opacity: 0.95,
    depthTest: false,
    depthWrite: false,
    toneMapped: false,
  });
}

function flatPoints(points: readonly THREE.Vector2[]): number[] {
  return points.flatMap((point) => [point.x, point.y, 0]);
}

function pickMaterial(): THREE.MeshBasicMaterial {
  return new THREE.MeshBasicMaterial({
    visible: false,
    transparent: true,
    opacity: 0,
    depthTest: false,
    depthWrite: false,
  });
}

/** Cylinder/cone geometries point along +Y; turn the group so +Y is the axis. */
function orientAlong(group: THREE.Group, axis: PartGizmoAxis): void {
  group.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), AXIS_VECTORS[axis]);
}

function setWorldPosition(object: THREE.Object3D, world: THREE.Vector3): void {
  if (object.parent) {
    object.parent.updateWorldMatrix(true, false);
    object.position.copy(object.parent.worldToLocal(world.clone()));
  } else {
    object.position.copy(world);
  }
}

function setWorldQuaternion(object: THREE.Object3D, world: THREE.Quaternion): void {
  if (object.parent) {
    const parentInverse = object.parent.getWorldQuaternion(new THREE.Quaternion()).invert();
    object.quaternion.copy(parentInverse.multiply(world));
  } else {
    object.quaternion.copy(world);
  }
}
