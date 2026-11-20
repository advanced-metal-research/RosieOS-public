import * as THREE from "three";
import type { Vec3 } from "../domain/project.ts";
import {
  pointOnPolylineNormalizedS,
  samplePolylineNormalizedWindow,
} from "../domain/seam.ts";
import { disposeObjectTree } from "./resource-disposal.ts";

/**
 * Pickable CAD topology edges for the workstation stage (P2.12).
 *
 * Copied from GradientOS `web-ui/src/ArmVisualizer.tsx`:
 *   - every B-rep edge is a `THREE.Line` (LineBasicMaterial)
 *   - an invisible 0.5 mm TubeGeometry is the raycast target
 *   - hover/active add a visible tube at 0.5 mm / 0.6 mm
 * Face-wire composites are filtered before they reach this layer so a click
 * is one line, not every edge on a surface.
 */

export interface EdgeOverlayEdge {
  id: string;
  /** World-space polyline for the edge (already fixture-placed). */
  pointsM: readonly Vec3[];
  closed: boolean;
}

export interface EdgeWeldWindow {
  startS: number;
  endS: number;
}

export interface EdgeSelectionState {
  /** Edge-selection mode: hover styling and click-to-select are active. */
  mode: boolean;
  selectedIds: ReadonlySet<string>;
  activeId: string | null;
  /** Normalized trim on the active edge. GradientOS weldStart/Stop. */
  activeWindow?: EdgeWeldWindow | null;
}

export interface EdgePickingCallbacks {
  /** Left click on an edge while edge-selection mode is on. The pick-ray's
   *  hit point on the edge tube rides along, so a mode can care WHERE along
   *  the edge the click landed, not just which edge. */
  onEdgeClick: (edgeId: string, pointM?: Vec3) => void;
  /** Shift + left click anywhere: picked scene position for add-point. */
  onScenePointClick: (pointM: Vec3) => void;
  /** Plain click in edge-selection mode that hit NO edge pick tube — the
   *  picked scene position (surface first, then table plane), so a mode can
   *  offer a nearest-feature fallback instead of silence. Optional: absent,
   *  a missed click stays inert as it always was. */
  onEdgeMissClick?: (pointM: Vec3 | null) => void;
  /** A plain click that landed on the part's surface, in or out of
   *  edge-selection mode (after the edge click when one also hit): the
   *  part is a thing to select, like a row in the tree. */
  onPartClick?: (pointM: Vec3) => void;
  onHoverChange?: (edgeId: string | null) => void;
}

export type EdgeVisualState =
  | "active"
  | "selected"
  | "hover"
  | "default";

/** GradientOS ArmVisualizer.tsx palette and tube radii. */
export const EDGE_STATE_STYLE: Readonly<
  Record<EdgeVisualState, { color: number; opacity: number; renderOrder: number }>
> = {
  default: { color: 0x3f5669, opacity: 0.65, renderOrder: 11 },
  hover: { color: 0x1a45c4, opacity: 1, renderOrder: 17 },
  selected: { color: 0x2a6238, opacity: 1, renderOrder: 16 },
  active: { color: 0x2a6238, opacity: 1, renderOrder: 18 },
};

/**
 * Invisible pick corridor. GradientOS uses 0.5 mm because that camera sits on
 * the part. OLP's default orbit is the robot cell, where 0.5 mm is sub-pixel
 * and unhittable. Keep this fat; the visible hover/selected tubes stay thin.
 */
export const EDGE_PICK_RADIUS_M = 0.004;
/** Visible hover tube. GradientOS `TOPOLOGY_EDGE_PICK_RADIUS_M` (0.5 mm). */
export const EDGE_HOVER_RADIUS_M = 0.0005;
/** Visible active-edge tube. GradientOS `TOPOLOGY_EDGE_SELECTED_RADIUS_M`. */
export const EDGE_SELECTED_RADIUS_M = 0.0006;
/** Start/stop spheres. GradientOS `ArmVisualizer.tsx` weld-start-stop-markers. */
export const WELD_ENDPOINT_RADIUS_M = 0.0075;
export const WELD_START_COLOR = 0x2a6238;
export const WELD_STOP_COLOR = 0xa8231c;
export const WELD_WINDOW_LINE_COLOR = 0x7f4200;
/** Pointer travel below this many CSS pixels counts as a click, not a drag. */
export const EDGE_CLICK_SLOP_PX = 5;

export function edgeVisualState(
  edge: { id: string },
  state: EdgeSelectionState,
  hoveredId: string | null,
): EdgeVisualState {
  if (state.activeId === edge.id && state.selectedIds.has(edge.id)) return "active";
  if (state.selectedIds.has(edge.id)) return "selected";
  if (state.mode && hoveredId === edge.id) return "hover";
  return "default";
}

/** NDC coordinates of a pointer event relative to a pane rectangle. */
export function pointerNDC(
  clientX: number,
  clientY: number,
  rect: { left: number; top: number; width: number; height: number },
): { x: number; y: number } | null {
  if (rect.width <= 0 || rect.height <= 0) return null;
  const x = ((clientX - rect.left) / rect.width) * 2 - 1;
  const y = -((clientY - rect.top) / rect.height) * 2 + 1;
  if (x < -1 || x > 1 || y < -1 || y > 1) return null;
  return { x, y };
}

interface EdgeRecord {
  edge: EdgeOverlayEdge;
  line: THREE.Line;
  pick: THREE.Mesh;
}

export interface EdgeSelectionControllerOptions {
  /** Where the pick tubes live — the work-frame follower, so they ride the
   *  positioner with the part they outline. */
  scene: THREE.Object3D;
  camera: THREE.PerspectiveCamera;
  /** Element the orbit view renders into; pointer events are read here. */
  element: HTMLElement;
  /** Put every drawn and pickable object on this layer only, for a picker
   *  that serves a camera on that layer (the seam window's) and must not
   *  be seen by the main one. */
  layer?: number;
  /** World Z of the table top — the shift-click fallback pick plane. */
  groundZ: number;
  callbacks: EdgePickingCallbacks;
  /** Scene object names raycast for the shift-click surface pick. */
  surfaceLayerNames?: readonly string[];
}

export class EdgeSelectionController {
  private readonly scene: THREE.Object3D;
  private readonly camera: THREE.PerspectiveCamera;
  private readonly element: HTMLElement;
  private readonly layer: number | null;
  private readonly callbacks: EdgePickingCallbacks;
  private readonly surfaceLayerNames: readonly string[];
  private readonly group = new THREE.Group();
  private readonly raycaster = new THREE.Raycaster();
  private readonly pointer = new THREE.Vector2();
  private readonly groundPlane: THREE.Plane;
  private records = new Map<string, EdgeRecord>();
  private state: EdgeSelectionState = {
    mode: false,
    selectedIds: new Set(),
    activeId: null,
    activeWindow: null,
  };
  private hoveredId: string | null = null;
  private hoverOverlay: THREE.Mesh | null = null;
  private activeOverlay: THREE.Mesh | null = null;
  private weldEndpoints: THREE.Group | null = null;
  private pointerDown: { x: number; y: number; button: number; shiftKey: boolean } | null = null;
  private disposed = false;
  /** True while another pointer owner (the part gizmo) holds the pane: no
   *  hover, and the release that ends its drag is not a click. Set by the
   *  owner around its drag; this controller registered its listeners first,
   *  so it sees the release while the flag is still up. */
  suppressed = false;

  constructor(options: EdgeSelectionControllerOptions) {
    this.scene = options.scene;
    this.camera = options.camera;
    this.element = options.element;
    this.layer = options.layer ?? null;
    this.callbacks = options.callbacks;
    this.surfaceLayerNames = options.surfaceLayerNames ??
      ["workstation-workpiece"];
    this.groundPlane = new THREE.Plane(
      new THREE.Vector3(0, 0, 1),
      -options.groundZ,
    );
    this.group.name = "workstation-cad-edges";
    this.scene.add(this.group);
    this.element.addEventListener("pointerdown", this.onPointerDown);
    this.element.addEventListener("pointermove", this.onPointerMove);
    this.element.addEventListener("pointerup", this.onPointerUp);
    this.element.addEventListener("pointerleave", this.onPointerLeave);
  }

  /** Rebuild the edge layer. Call when the catalog or placement changes. */
  setEdges(edges: readonly EdgeOverlayEdge[]): void {
    if (this.disposed) return;
    this.clearOverlays();
    for (const record of this.records.values()) {
      this.group.remove(record.line, record.pick);
      disposeObjectTree(record.line);
      disposeObjectTree(record.pick);
    }
    this.records = new Map();
    this.hoveredId = null;
    for (const edge of edges) {
      if (edge.pointsM.length < 2) continue;
      const points = edge.pointsM.map((point) => new THREE.Vector3(...point));
      const line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(points),
        new THREE.LineBasicMaterial({
          color: EDGE_STATE_STYLE.default.color,
          transparent: true,
          opacity: EDGE_STATE_STYLE.default.opacity,
          depthTest: false,
          depthWrite: false,
          toneMapped: false,
        }),
      );
      line.name = `cad-edge-${safeName(edge.id)}`;
      line.renderOrder = EDGE_STATE_STYLE.default.renderOrder;
      line.userData.edgeId = edge.id;
      const pick = new THREE.Mesh(
        edgeTubeGeometry(points, EDGE_PICK_RADIUS_M, edge.closed),
        new THREE.MeshBasicMaterial({
          visible: false,
          transparent: true,
          opacity: 0,
          depthTest: false,
          depthWrite: false,
        }),
      );
      pick.name = `cad-edge-pick-${safeName(edge.id)}`;
      pick.renderOrder = 10;
      pick.userData.edgeId = edge.id;
      this.group.add(line, pick);
      this.records.set(edge.id, { edge, line, pick });
    }
    this.element.dataset.cadEdgeCount = String(this.records.size);
    this.refreshVisuals();
  }

  setState(state: EdgeSelectionState): void {
    if (this.disposed) return;
    this.state = {
      mode: state.mode,
      selectedIds: new Set(state.selectedIds),
      activeId: state.activeId,
      activeWindow: state.activeWindow ?? null,
    };
    if (!state.mode) this.setHovered(null);
    this.element.dataset.cadEdgeMode = state.mode ? "select" : "off";
    this.element.dataset.cadEdgeSelected = [...state.selectedIds].join(",");
    const window = state.activeWindow;
    this.element.dataset.cadWeldWindow = window
      ? `${window.startS.toFixed(3)},${window.endS.toFixed(3)}`
      : "";
    this.refreshVisuals();
  }

  hasEdges(): boolean {
    return this.records.size > 0;
  }

  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;
    this.element.removeEventListener("pointerdown", this.onPointerDown);
    this.element.removeEventListener("pointermove", this.onPointerMove);
    this.element.removeEventListener("pointerup", this.onPointerUp);
    this.element.removeEventListener("pointerleave", this.onPointerLeave);
    this.clearOverlays();
    this.scene.remove(this.group);
    disposeObjectTree(this.group);
    this.group.clear();
    this.records = new Map();
  }

  private onPointerDown = (event: PointerEvent): void => {
    // Shift+click is the add-point gesture; without this the browser treats
    // it as extend-text-selection and smears highlights across the UI.
    if (event.button === 0 && event.shiftKey) event.preventDefault();
    this.pointerDown = {
      x: event.clientX,
      y: event.clientY,
      button: event.button,
      shiftKey: event.shiftKey,
    };
  };

  private onPointerMove = (event: PointerEvent): void => {
    if (this.suppressed) return;
    if (!this.state.mode || this.records.size === 0) return;
    // While a drag is in flight the operator is orbiting, not picking.
    if (this.pointerDown && !withinClickSlop(this.pointerDown, event)) {
      this.setHovered(null);
      return;
    }
    this.setHovered(this.pickEdgeId(event));
  };

  private onPointerUp = (event: PointerEvent): void => {
    const started = this.pointerDown;
    this.pointerDown = null;
    if (this.suppressed) return;
    if (!started || started.button !== 0 || event.button !== 0) return;
    if (!withinClickSlop(started, event)) return;
    if (started.shiftKey || event.shiftKey) {
      const pointM = this.pickScenePoint(event);
      if (pointM) this.callbacks.onScenePointClick(pointM);
      return;
    }
    const partPoint = this.pickPartPoint(event.clientX, event.clientY);
    if (!this.state.mode) {
      if (partPoint) this.callbacks.onPartClick?.(partPoint);
      return;
    }
    const hit = this.pickEdgeHit(event);
    if (hit) {
      this.setHovered(hit.edgeId);
      this.callbacks.onEdgeClick(hit.edgeId, hit.pointM);
    } else {
      if (partPoint) this.callbacks.onPartClick?.(partPoint);
      this.callbacks.onEdgeMissClick?.(this.pickScenePoint(event));
    }
  };

  /** Where a screen point lands on the part's surface, or null: for a
   *  click or a right-click that should mean the part. */
  pickPartPoint(clientX: number, clientY: number): Vec3 | null {
    const ndc = pointerNDC(clientX, clientY, this.element.getBoundingClientRect());
    if (!ndc) return null;
    this.pointer.set(ndc.x, ndc.y);
    this.raycaster.setFromCamera(this.pointer, this.camera);
    const surfaces = this.surfaceLayerNames.flatMap((name) => {
      const layer = this.scene.getObjectByName(name);
      return layer ? [layer] : [];
    });
    const hit = surfaces.length > 0 ? this.raycaster.intersectObjects(surfaces, true)[0] : undefined;
    return hit ? [hit.point.x, hit.point.y, hit.point.z] : null;
  }

  private onPointerLeave = (): void => {
    this.pointerDown = null;
    this.setHovered(null);
  };

  private updateRaycaster(event: PointerEvent): boolean {
    const ndc = pointerNDC(
      event.clientX,
      event.clientY,
      this.element.getBoundingClientRect(),
    );
    if (!ndc) return false;
    this.pointer.set(ndc.x, ndc.y);
    this.raycaster.setFromCamera(this.pointer, this.camera);
    return true;
  }

  private pickEdgeId(event: PointerEvent): string | null {
    return this.pickEdgeHit(event)?.edgeId ?? null;
  }

  private pickEdgeHit(event: PointerEvent): { edgeId: string; pointM: Vec3 } | null {
    if (!this.updateRaycaster(event)) return null;
    const meshes = [...this.records.values()].map((record) => record.pick);
    const hit = this.raycaster.intersectObjects(meshes, false)[0];
    const edgeId = hit?.object.userData.edgeId;
    if (typeof edgeId !== "string" || !hit) return null;
    return { edgeId, pointM: [hit.point.x, hit.point.y, hit.point.z] };
  }

  /** Shift-click pick: part surface first, then edges, then the table plane. */
  private pickScenePoint(event: PointerEvent): Vec3 | null {
    if (!this.updateRaycaster(event)) return null;
    const surfaces = this.surfaceLayerNames.flatMap((name) => {
      const layer = this.scene.getObjectByName(name);
      return layer ? [layer] : [];
    });
    const surfaceHit = surfaces.length > 0
      ? this.raycaster.intersectObjects(surfaces, true)[0]
      : undefined;
    if (surfaceHit) return [surfaceHit.point.x, surfaceHit.point.y, surfaceHit.point.z];
    const meshes = [...this.records.values()].map((record) => record.pick);
    const edgeHit = meshes.length > 0
      ? this.raycaster.intersectObjects(meshes, false)[0]
      : undefined;
    if (edgeHit) return [edgeHit.point.x, edgeHit.point.y, edgeHit.point.z];
    const intersection = new THREE.Vector3();
    if (this.raycaster.ray.intersectPlane(this.groundPlane, intersection)) {
      return [intersection.x, intersection.y, intersection.z];
    }
    return null;
  }

  private setHovered(edgeId: string | null): void {
    if (this.hoveredId === edgeId) return;
    this.hoveredId = edgeId;
    this.element.dataset.cadEdgeHover = edgeId ?? "";
    this.callbacks.onHoverChange?.(edgeId);
    this.refreshVisuals();
  }

  /** The group holding the lines and pick tubes, for a view that must leave
   *  it out. */
  get layerGroup(): THREE.Group {
    return this.group;
  }

  private refreshVisuals(): void {
    this.clearOverlays();
    for (const record of this.records.values()) {
      const visual = edgeVisualState(record.edge, this.state, this.hoveredId);
      const style = EDGE_STATE_STYLE[visual];
      const material = record.line.material as THREE.LineBasicMaterial;
      material.color.setHex(style.color);
      material.opacity = style.opacity;
      material.transparent = style.opacity < 1;
      record.line.renderOrder = style.renderOrder;
    }
    // GradientOS: hover tube only when the hovered edge is not already selected.
    if (
      this.state.mode &&
      this.hoveredId &&
      !this.state.selectedIds.has(this.hoveredId)
    ) {
      const hovered = this.records.get(this.hoveredId);
      if (hovered) {
        this.hoverOverlay = this.addOverlayTube(
          hovered,
          EDGE_HOVER_RADIUS_M,
          EDGE_STATE_STYLE.hover.color,
          24,
        );
      }
    }
    // GradientOS: one selected-radius tube on the active edge only.
    if (this.state.activeId) {
      const active = this.records.get(this.state.activeId);
      if (active) {
        this.activeOverlay = this.addOverlayTube(
          active,
          EDGE_SELECTED_RADIUS_M,
          EDGE_STATE_STYLE.active.color,
          24,
        );
        this.weldEndpoints = this.addWeldEndpoints(active);
      }
    }
    if (this.layer !== null) {
      const layer = this.layer;
      this.group.traverse((node) => node.layers.set(layer));
    }
  }

  /**
   * Green start sphere, red stop sphere, amber kept-span line — copied from
   * GradientOS `ArmVisualizer.tsx` `weld-start-stop-markers`.
   */
  private addWeldEndpoints(record: EdgeRecord): THREE.Group | null {
    const window = this.state.activeWindow;
    const startS = window?.startS ?? 0;
    const endS = window?.endS ?? 1;
    const start = pointOnPolylineNormalizedS(record.edge.pointsM, startS);
    const stop = pointOnPolylineNormalizedS(record.edge.pointsM, endS);
    if (!start || !stop) return null;
    const group = new THREE.Group();
    group.name = "weld-start-stop-markers";
    const startMarker = new THREE.Mesh(
      new THREE.SphereGeometry(WELD_ENDPOINT_RADIUS_M, 16, 16),
      new THREE.MeshBasicMaterial({
        color: WELD_START_COLOR,
        depthTest: false,
        depthWrite: false,
        toneMapped: false,
      }),
    );
    const stopMarker = new THREE.Mesh(
      new THREE.SphereGeometry(WELD_ENDPOINT_RADIUS_M, 16, 16),
      new THREE.MeshBasicMaterial({
        color: WELD_STOP_COLOR,
        depthTest: false,
        depthWrite: false,
        toneMapped: false,
      }),
    );
    startMarker.position.set(...start);
    stopMarker.position.set(...stop);
    startMarker.name = "weld-start-marker";
    stopMarker.name = "weld-stop-marker";
    startMarker.renderOrder = 30;
    stopMarker.renderOrder = 30;
    group.add(startMarker, stopMarker);
    const segment = samplePolylineNormalizedWindow(
      record.edge.pointsM,
      startS,
      endS,
      28,
    );
    if (segment.length >= 2) {
      const line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(
          segment.map((point) => new THREE.Vector3(...point)),
        ),
        new THREE.LineBasicMaterial({
          color: WELD_WINDOW_LINE_COLOR,
          transparent: true,
          opacity: 0.9,
          depthTest: false,
          depthWrite: false,
          toneMapped: false,
        }),
      );
      line.name = "weld-window-line";
      line.renderOrder = 29;
      group.add(line);
    }
    this.group.add(group);
    return group;
  }

  private addOverlayTube(
    record: EdgeRecord,
    radiusM: number,
    color: number,
    renderOrder: number,
  ): THREE.Mesh {
    const points = record.edge.pointsM.map((point) => new THREE.Vector3(...point));
    const overlay = new THREE.Mesh(
      edgeTubeGeometry(points, radiusM, record.edge.closed),
      new THREE.MeshBasicMaterial({
        color,
        transparent: true,
        opacity: 0.95,
        depthTest: false,
        depthWrite: false,
        toneMapped: false,
      }),
    );
    overlay.name = `cad-edge-overlay-${safeName(record.edge.id)}`;
    overlay.renderOrder = renderOrder;
    this.group.add(overlay);
    return overlay;
  }

  private clearOverlays(): void {
    for (const overlay of [this.hoverOverlay, this.activeOverlay, this.weldEndpoints]) {
      if (!overlay) continue;
      this.group.remove(overlay);
      disposeObjectTree(overlay);
    }
    this.hoverOverlay = null;
    this.activeOverlay = null;
    this.weldEndpoints = null;
  }
}

function edgeTubeGeometry(
  points: THREE.Vector3[],
  radiusM: number,
  closed: boolean,
): THREE.TubeGeometry {
  const curve = new THREE.CatmullRomCurve3(points, closed, "centripetal");
  const tubularSegments = Math.min(512, Math.max(24, points.length * 4));
  return new THREE.TubeGeometry(curve, tubularSegments, radiusM, 10, closed);
}

function withinClickSlop(
  start: { x: number; y: number },
  event: PointerEvent,
): boolean {
  return Math.hypot(event.clientX - start.x, event.clientY - start.y) <=
    EDGE_CLICK_SLOP_PX;
}

function safeName(value: string): string {
  return value.replace(/[^a-zA-Z0-9_-]+/g, "-");
}
