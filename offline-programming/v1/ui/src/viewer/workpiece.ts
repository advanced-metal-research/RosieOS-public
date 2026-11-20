import * as THREE from "three";

import type { CadWorkpieceMesh } from "../cad/contracts.ts";
import type { Rpy, Vec3 } from "../domain/project.ts";

/**
 * Placement of the workpiece in the workstation world frame. Because the CAD →
 * world map is rigid, the whole mesh rides one group transform rather than
 * every vertex being rewritten on each placement edit.
 *
 * Both fields come straight from domain/placement.ts:
 *   originM  = transformWorkpiecePoint(project, [0, 0, 0])
 *   rotationRpyRad = workpieceWorldRotationRpy(project)
 */
export interface WorkpieceMeshPlacement {
  originM: Vec3;
  rotationRpyRad: Rpy;
  /** The placement anchor in the world (workpieceAnchorWorldM): where the
   *  drag gizmo pivots. Optional so an overlay-only caller need not compute it. */
  anchorM?: Vec3;
}

/**
 * Steel-like neutral. Deliberately desaturated: the stage background is
 * shop grey (0xcfd2d5) and every overlay is a semantic colour (weld amber, travel
 * green, weave amber, torch cyan/orange), so the part has to read as material
 * rather than as another annotation colour.
 *
 * Path and TCP overlays are drawn with depthTest disabled at renderOrder 100+,
 * so an opaque part cannot occlude them.
 */
const WORKPIECE_COLOR = 0xe6e8ea;
const WORKPIECE_EDGE_COLOR = 0x4a5259;
const WORKPIECE_LAYER_NAME = "workstation-workpiece";

export interface WorkpieceMeshOptions {
  /** Adds a subtle wireframe over the surface; off by default. */
  showEdges?: boolean;
  opacity?: number;
}

/**
 * Build the workpiece layer. Returns an empty (but correctly named) group when
 * there is no mesh, so the caller can always swap layers through the renderer's
 * replaceLayer path and never has to branch on null.
 */
export function createWorkpieceMeshGroup(
  mesh: CadWorkpieceMesh | null,
  placement: WorkpieceMeshPlacement | null,
  options: WorkpieceMeshOptions = {},
): THREE.Group {
  const group = new THREE.Group();
  group.name = WORKPIECE_LAYER_NAME;
  if (!mesh || mesh.indices.length === 0 || mesh.positionsM.length === 0) {
    return group;
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    "position",
    new THREE.BufferAttribute(mesh.positionsM, 3),
  );
  geometry.setIndex(new THREE.BufferAttribute(mesh.indices, 1));
  if (mesh.normals && mesh.normals.length === mesh.positionsM.length) {
    geometry.setAttribute("normal", new THREE.BufferAttribute(mesh.normals, 3));
  } else {
    // OCCT split the triangulation per face, so averaging shared vertices here
    // does not round off the part's corners.
    geometry.computeVertexNormals();
  }
  geometry.computeBoundingSphere();

  const opacity = clamp01(options.opacity ?? 1);
  const material = new THREE.MeshStandardMaterial({
    color: WORKPIECE_COLOR,
    metalness: 0.35,
    roughness: 0.55,
    flatShading: false,
    side: THREE.DoubleSide,
    transparent: opacity < 1,
    opacity,
    // Keeps the optional wireframe from z-fighting the surface it sits on.
    polygonOffset: true,
    polygonOffsetFactor: 1,
    polygonOffsetUnits: 1,
  });

  const surface = new THREE.Mesh(geometry, material);
  surface.name = `${WORKPIECE_LAYER_NAME}-surface`;
  surface.castShadow = true;
  surface.receiveShadow = true;
  surface.userData.triangleCount = mesh.triangleCount;
  surface.userData.fingerprint = mesh.fingerprint;
  group.add(surface);

  if (options.showEdges === true) {
    const edges = new THREE.LineSegments(
      new THREE.WireframeGeometry(geometry),
      new THREE.LineBasicMaterial({
        color: WORKPIECE_EDGE_COLOR,
        transparent: true,
        opacity: 0.35 * opacity,
        toneMapped: false,
      }),
    );
    edges.name = `${WORKPIECE_LAYER_NAME}-edges`;
    group.add(edges);
  }

  applyWorkpiecePlacement(group, placement);
  group.userData.fingerprint = mesh.fingerprint;
  group.userData.triangleCount = mesh.triangleCount;
  group.userData.truncated = mesh.truncated;
  return group;
}

/**
 * Apply the rigid CAD → world transform. Identity when there is no placement,
 * which is what an un-fixtured project should look like.
 */
export function applyWorkpiecePlacement(
  group: THREE.Object3D,
  placement: WorkpieceMeshPlacement | null,
): void {
  if (!placement) {
    group.position.set(0, 0, 0);
    group.rotation.set(0, 0, 0, "XYZ");
    return;
  }
  const [x, y, z] = placement.originM;
  const [roll, pitch, yaw] = placement.rotationRpyRad;
  group.position.set(finite(x), finite(y), finite(z));
  // "ZYX", because workpieceWorldRotationRpy decomposes the world rotation in
  // that order (Rz · Ry · Rx, the URDF fixed-axis convention the row/TCP
  // contract uses). Reading the same three numbers back as "XYZ" composes
  // Rx · Ry · Rz instead: identical for a pure yaw, which is all the old test
  // tried, and 50° off for roll 0.4 / pitch -0.7 / yaw 1.1 (2026-09-13).
  group.rotation.set(finite(roll), finite(pitch), finite(yaw), "ZYX");
}

/**
 * Identity of a built workpiece layer: same string means the existing geometry
 * can be reused and only its placement needs updating. Triangle count is
 * included so a re-tessellation at a different deflection still rebuilds.
 */
export function workpieceMeshIdentity(
  mesh: CadWorkpieceMesh,
  options: WorkpieceMeshOptions = {},
): string {
  return [
    mesh.fingerprint,
    mesh.triangleCount,
    mesh.vertexCount,
    options.showEdges === true ? "edges" : "solid",
    options.opacity ?? 1,
  ].join(":");
}

/** One-line viewport summary for the status bar. */
export function workpieceMeshSummary(mesh: CadWorkpieceMesh): string {
  const triangles = mesh.triangleCount.toLocaleString("en-US");
  const size = mesh.boundsMaxM.map((value, index) =>
    Math.round((value - mesh.boundsMinM[index]!) * 1000)
  );
  const extent = size.some((value) => value > 0) ? ` · ${size.join(" × ")} mm` : "";
  const capped = mesh.truncated ? " · truncated at the triangle cap" : "";
  return `Workpiece mesh · ${triangles} triangles${extent}${capped}`;
}

function clamp01(value: number): number {
  return Number.isFinite(value) ? Math.max(0, Math.min(1, value)) : 1;
}

function finite(value: number): number {
  return Number.isFinite(value) ? value : 0;
}
