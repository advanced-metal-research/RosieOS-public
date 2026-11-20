/**
 * Waypoint welds in the viewport: the path of each, and while the tool is
 * open a torch at every waypoint, in the part's frame (the caller parents
 * the root under the renderer's `waypointLayer`). The selected weld is drawn
 * solid; the others step back.
 *
 * Torches are kept per waypoint id and only re-posed on a sync, never
 * rebuilt, because the handle is attached to one: rebuilding the object
 * under a held handle would leave the handle driving a detached copy.
 */

import * as THREE from "three";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";

import { samplePath, type Waypoint, type WeldShape } from "./model.ts";
import { shapeOf, type WaypointWeld } from "./node.ts";
import type { WaypointSelection } from "./session.ts";

/** The accent blue, as the rails direction arrows: a waypoint weld is
 *  something the operator drew, not something detected (amber). */
const PATH_COLOR = 0x1a45c4;
const PATH_WIDTH_PX = 2;
const TORCH_COLOR = 0x8f979e;
const TORCH_SELECTED_COLOR = 0xe08a2e;
const TIP_COLOR = 0x1a45c4;
/** How much of itself an unselected weld keeps. */
const UNSELECTED_OPACITY = 0.35;

/** A generic MIG gun, in metres, contact tip at its origin and the body up
 *  local -Z: a contact tip poking 3 mm out of a 16 mm nozzle, then the
 *  barrel. Proportions to be recognisable, not the fitted gun's -- that
 *  lives in the rails torch parameters and is not what this glyph is for.
 *  The gun is drawn the waypoint's standoff back from its work point. */
const CONTACT_TIP = { radius: 0.002, from: 0, to: 0.004 };
const NOZZLE = { radiusTip: 0.006, radiusBase: 0.008, from: 0.003, to: 0.035 };
const BARREL = { radius: 0.011, from: 0.035, to: 0.09 };
const WORK_POINT_RADIUS_M = 0.0015;

interface TorchRecord {
  /** At the work point, with the torch's orientation: what the handle holds. */
  group: THREE.Group;
  /** The gun, slid back along -Z by the standoff. */
  gun: THREE.Group;
  /** The standoff gap, work point to contact tip. */
  gap: THREE.Line;
  materials: THREE.MeshStandardMaterial[];
}

export class WaypointOverlay {
  readonly root = new THREE.Group();
  readonly #torches = new Map<string, TorchRecord>();
  readonly #torchGroup = new THREE.Group();
  readonly #paths = new THREE.Group();

  constructor() {
    this.root.name = "waypoint-welds";
    this.#torchGroup.name = "waypoint-torches";
    this.#paths.name = "waypoint-paths";
    this.root.add(this.#paths, this.#torchGroup);
  }

  /** The torch drawn for a waypoint, for the handle to hold. */
  torchFor(waypointId: string | null): THREE.Object3D | null {
    return waypointId ? this.#torches.get(waypointId)?.group ?? null : null;
  }

  /** Torches are shown only while the tool is open; paths always. */
  set torchesVisible(visible: boolean) {
    this.#torchGroup.visible = visible;
  }

  sync(welds: readonly WaypointWeld[], selection: WaypointSelection): void {
    const alive = new Set<string>();
    for (const weld of welds) {
      const opacity = weld.id === selection.weldId ? 1 : UNSELECTED_OPACITY;
      for (const point of weld.waypoints) {
        alive.add(point.id);
        let torch = this.#torches.get(point.id);
        if (!torch) {
          torch = buildTorch();
          torch.group.name = `waypoint-torch-${point.id}`;
          this.#torches.set(point.id, torch);
          this.#torchGroup.add(torch.group);
        }
        torch.group.position.set(...point.positionM);
        torch.group.quaternion.set(...point.quaternion);
        const standoffM = point.standoffMm / 1_000;
        torch.gun.position.z = -standoffM;
        const gap = torch.gap.geometry.getAttribute("position") as THREE.BufferAttribute;
        gap.setZ(1, -standoffM);
        gap.needsUpdate = true;
        torch.gap.geometry.computeBoundingSphere();
        const color = point.id === selection.waypointId ? TORCH_SELECTED_COLOR : TORCH_COLOR;
        for (const material of torch.materials) {
          if (material.userData.role === "body") material.color.setHex(color);
          material.opacity = opacity;
          material.transparent = opacity < 1;
          material.depthWrite = opacity >= 1;
        }
      }
    }
    for (const [id, torch] of this.#torches) {
      if (alive.has(id)) continue;
      disposeTree(torch.group);
      this.#torches.delete(id);
    }
    this.#drawPaths(welds, selection.weldId);
  }

  /**
   * Redraw only the paths, with one waypoint at a pose that is not stored
   * yet -- the one under a held handle. The torches are left alone: the
   * handle already moves the held one, and the rest have not moved.
   */
  previewPaths(
    welds: readonly WaypointWeld[],
    selectedWeldId: string | null,
    moving: Pick<Waypoint, "id" | "positionM" | "quaternion">,
  ): void {
    this.#drawPaths(
      welds.map((weld) => ({
        ...weld,
        waypoints: weld.waypoints.map((point) => (point.id === moving.id ? { ...point, ...moving } : point)),
      })),
      selectedWeldId,
    );
  }

  #drawPaths(welds: readonly WaypointWeld[], selectedWeldId: string | null): void {
    for (const child of [...this.#paths.children]) disposeTree(child);
    for (const weld of welds) {
      if (weld.waypoints.length >= 2) {
        this.#paths.add(buildPath(weld.waypoints, shapeOf(weld.tool), weld.id === selectedWeldId));
      }
    }
  }

  dispose(): void {
    disposeTree(this.root);
    this.#torches.clear();
  }
}

function buildTorch(): TorchRecord {
  const group = new THREE.Group();
  const gun = new THREE.Group();
  const body = (role: string, color: number) => {
    const material = new THREE.MeshStandardMaterial({ color, metalness: 0.3, roughness: 0.55 });
    material.userData.role = role;
    return material;
  };
  const bodyMaterial = body("body", TORCH_COLOR);
  const tipMaterial = body("tip", TIP_COLOR);
  // Cylinders are built along +Y; each part is turned onto -Z and slid back
  // from the contact tip by its own span.
  const cylinder = (radiusTip: number, radiusBase: number, from: number, to: number, material: THREE.Material) => {
    const geometry = new THREE.CylinderGeometry(radiusBase, radiusTip, to - from, 24);
    const mesh = new THREE.Mesh(geometry, material);
    mesh.rotation.x = -Math.PI / 2;
    mesh.position.z = -(from + to) / 2;
    return mesh;
  };
  gun.add(
    cylinder(CONTACT_TIP.radius, CONTACT_TIP.radius, CONTACT_TIP.from, CONTACT_TIP.to, tipMaterial),
    cylinder(NOZZLE.radiusTip, NOZZLE.radiusBase, NOZZLE.from, NOZZLE.to, bodyMaterial),
    cylinder(BARREL.radius, BARREL.radius, BARREL.from, BARREL.to, bodyMaterial),
  );
  const gapGeometry = new THREE.BufferGeometry();
  gapGeometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(6), 3));
  const gap = new THREE.Line(gapGeometry, new THREE.LineBasicMaterial({ color: TIP_COLOR }));
  group.add(
    gun,
    gap,
    new THREE.Mesh(new THREE.SphereGeometry(WORK_POINT_RADIUS_M, 12, 8), tipMaterial),
  );
  return { group, gun, gap, materials: [bodyMaterial, tipMaterial] };
}

function buildPath(waypoints: readonly Waypoint[], shape: WeldShape, selected: boolean): THREE.Object3D {
  const positions = samplePath(waypoints, 16, shape).flat();
  const material = new LineMaterial({
    color: PATH_COLOR,
    linewidth: PATH_WIDTH_PX,
    worldUnits: false,
    transparent: !selected,
    opacity: selected ? 1 : UNSELECTED_OPACITY,
  });
  const line = new Line2(new LineGeometry().setPositions(positions), material);
  line.name = "waypoint-path";
  line.computeLineDistances();
  // Fat lines measure their width against the pane they are drawn into.
  const viewport = new THREE.Vector4();
  line.onBeforeRender = (renderer) => {
    renderer.getCurrentViewport(viewport);
    material.resolution.set(Math.max(1, viewport.z), Math.max(1, viewport.w));
    material.linewidth = PATH_WIDTH_PX * renderer.getPixelRatio();
  };
  line.renderOrder = 3;
  return line;
}

function disposeTree(object: THREE.Object3D): void {
  object.traverse((node) => {
    const mesh = node as THREE.Mesh;
    mesh.geometry?.dispose();
    const material = mesh.material as THREE.Material | THREE.Material[] | undefined;
    for (const owned of Array.isArray(material) ? material : material ? [material] : []) owned.dispose();
  });
  object.removeFromParent();
}
