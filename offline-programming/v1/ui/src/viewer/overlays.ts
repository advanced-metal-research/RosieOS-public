import * as THREE from "three";
import type {
  MotionClass,
  TcpPose,
  Vec3,
} from "../simulation/contracts.ts";
import {
  buildMotionRuns,
  type MotionPathPoint,
} from "./geometry.ts";

export interface OverlayPalette {
  travel: number;
  weld: number;
}

export interface SeamOverlay {
  id: string;
  points: readonly MotionPathPoint[];
  visible?: boolean;
}

export interface CollisionMarker {
  id: string;
  positionM: Vec3;
  severity?: "warning" | "collision";
  radiusM?: number;
  linkA?: string;
  linkB?: string;
}

export interface TcpFrameMarker {
  id: string;
  pose: TcpPose;
  sizeM?: number;
  visible?: boolean;
}

export interface CurveCenterMarker {
  id: string;
  positionM: Vec3;
  selected?: boolean;
}

// Shop grey: the planned motion in the confirmed green (weld) and the
// value blue-grey (travel); authored seams in caution amber, so a seam
// and the motion planned along it never read as the same line.
export const DEFAULT_PATH_PALETTE: Readonly<OverlayPalette> = {
  travel: 0x3f5669,
  weld: 0x2a6238,
};

export const DEFAULT_SEAM_PALETTE: Readonly<OverlayPalette> = {
  travel: 0x1a45c4,
  weld: 0x7f4200,
};

export function createMotionPathGroup(
  points: readonly MotionPathPoint[],
  palette: OverlayPalette = DEFAULT_PATH_PALETTE,
  name = "workstation-path",
  opacity = 1,
): THREE.Group {
  const group = new THREE.Group();
  group.name = name;
  const faded = Math.max(0, Math.min(1, opacity));
  for (const [index, run] of buildMotionRuns(points).entries()) {
    const geometry = new THREE.BufferGeometry().setFromPoints(
      run.pointsM.map((point) => new THREE.Vector3(...point)),
    );
    const material = new THREE.LineBasicMaterial({
      color: colorForMotion(run.motion, palette),
      transparent: faded < 1,
      opacity: faded,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
    });
    const line = new THREE.Line(geometry, material);
    line.name = `${name}-${run.motion}-${index}`;
    line.renderOrder = 100;
    line.userData.motion = run.motion;
    group.add(line);
  }
  for (const motion of ["travel", "weld"] as const) {
    const positions = points
      .filter((point) => point.motion === motion)
      .map((point) => new THREE.Vector3(...point.positionM));
    if (positions.length === 0) continue;
    const markers = new THREE.Points(
      new THREE.BufferGeometry().setFromPoints(positions),
      new THREE.PointsMaterial({
        color: colorForMotion(motion, palette),
        size: 8,
        sizeAttenuation: false,
        transparent: faded < 1,
        opacity: faded,
        depthTest: false,
        depthWrite: false,
        toneMapped: false,
      }),
    );
    markers.name = `${name}-${motion}-spreadsheet-targets`;
    markers.renderOrder = 101;
    markers.userData.motion = motion;
    markers.userData.spreadsheetTargets = true;
    group.add(markers);
  }
  return group;
}

/** Dense trajectories are lines, not thousands of authored waypoint dots.
 * Separate segments deliberately remain separate: gaps are not planned moves. */
export function createPlannedPathGroup(
  segments: readonly (readonly MotionPathPoint[])[],
  color?: number,
): THREE.Group {
  const group = new THREE.Group();
  group.name = "workstation-planned-tcp-path";
  for (const [segmentIndex, points] of segments.entries()) {
    for (const [runIndex, run] of buildMotionRuns(points).entries()) {
      const line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(run.pointsM.map(point => new THREE.Vector3(...point))),
        new THREE.LineBasicMaterial({
          color: color ?? (run.motion === "weld" ? DEFAULT_PATH_PALETTE.weld : 0x1a45c4),
          depthTest: false,
          depthWrite: false,
          toneMapped: false,
        }),
      );
      line.name = `planned-tcp-${segmentIndex}-${run.motion}-${runIndex}`;
      line.userData.motion = run.motion;
      line.renderOrder = 99;
      group.add(line);
    }
  }
  return group;
}

export function createSeamOverlayGroup(
  seams: readonly SeamOverlay[],
  palette: OverlayPalette = DEFAULT_SEAM_PALETTE,
): THREE.Group {
  const group = new THREE.Group();
  group.name = "workstation-seams";
  for (const seam of seams) {
    if (seam.visible === false) continue;
    const seamGroup = createMotionPathGroup(
      seam.points,
      palette,
      `workstation-seam-${safeName(seam.id)}`,
    );
    seamGroup.userData.seamId = seam.id;
    group.add(seamGroup);
  }
  return group;
}

export function createCollisionMarkerGroup(
  markers: readonly CollisionMarker[],
): THREE.Group {
  const group = new THREE.Group();
  group.name = "workstation-collisions";
  for (const marker of markers) {
    const radius = Math.max(0.005, marker.radiusM ?? 0.025);
    const color = marker.severity === "warning" ? 0x7f4200 : 0xa8231c;
    const markerGroup = new THREE.Group();
    markerGroup.name = `collision-${safeName(marker.id)}`;
    markerGroup.position.set(...marker.positionM);
    markerGroup.userData.collision = { ...marker };

    const sphere = new THREE.Mesh(
      new THREE.SphereGeometry(radius, 14, 10),
      new THREE.MeshBasicMaterial({
        color,
        transparent: true,
        opacity: 0.82,
        depthTest: false,
        toneMapped: false,
      }),
    );
    sphere.renderOrder = 120;
    markerGroup.add(sphere);

    const arm = radius * 1.8;
    const crossGeometry = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(-arm, 0, 0),
      new THREE.Vector3(arm, 0, 0),
      new THREE.Vector3(0, -arm, 0),
      new THREE.Vector3(0, arm, 0),
      new THREE.Vector3(0, 0, -arm),
      new THREE.Vector3(0, 0, arm),
    ]);
    const cross = new THREE.LineSegments(
      crossGeometry,
      new THREE.LineBasicMaterial({ color, depthTest: false, toneMapped: false }),
    );
    cross.renderOrder = 121;
    markerGroup.add(cross);
    group.add(markerGroup);
  }
  return group;
}

export function createTcpFrameGroup(
  frames: readonly TcpFrameMarker[],
): THREE.Group {
  const group = new THREE.Group();
  group.name = "workstation-tcp-frames";
  for (const frame of frames) {
    if (frame.visible === false) continue;
    const frameGroup = new THREE.Group();
    frameGroup.name = `tcp-frame-${safeName(frame.id)}`;
    applyTcpPose(frameGroup, frame.pose);
    const axes = new THREE.AxesHelper(Math.max(0.005, frame.sizeM ?? 0.08));
    axes.renderOrder = 110;
    disableMaterialDepth(axes.material);
    frameGroup.add(axes);
    group.add(frameGroup);
  }
  return group;
}

export function createCurveCenterGroup(
  centers: readonly CurveCenterMarker[],
): THREE.Group {
  const group = new THREE.Group();
  group.name = "workstation-curve-centers";
  for (const center of centers) {
    const marker = new THREE.Group();
    marker.name = `curve-center-${safeName(center.id)}`;
    marker.position.set(...center.positionM);
    marker.userData.curveCenter = true;
    marker.userData.moveId = center.id;
    const radius = center.selected ? 0.019 : 0.014;
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(radius, 0.003, 8, 20),
      new THREE.MeshBasicMaterial({
        color: center.selected ? 0x1a45c4 : 0x7f4200,
        depthTest: false,
        depthWrite: false,
        toneMapped: false,
      }),
    );
    ring.renderOrder = 114;
    marker.add(ring);
    const cross = new THREE.LineSegments(
      new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(-radius, 0, 0),
        new THREE.Vector3(radius, 0, 0),
        new THREE.Vector3(0, -radius, 0),
        new THREE.Vector3(0, radius, 0),
      ]),
      new THREE.LineBasicMaterial({
        color: 0x7f4200,
        depthTest: false,
        depthWrite: false,
        toneMapped: false,
      }),
    );
    cross.renderOrder = 115;
    marker.add(cross);
    group.add(marker);
  }
  return group;
}

export interface PathLabel {
  id: string;
  text: string;
  positionM: Vec3;
  motion: MotionClass;
  emphasis?: boolean;
}

export interface TorchGuideOverlay {
  originM: Vec3;
  travelDir: Vec3;
  sideDir: Vec3;
  torchUpDir: Vec3;
}

export interface SelectionOverlayInput {
  id: string;
  targetM: Vec3;
  previousM: Vec3 | null;
  motion: MotionClass;
  torchGuide: TorchGuideOverlay | null;
  weavePointsM: readonly Vec3[];
  /** Full-strength polyline for the run the selected row belongs to. */
  runPointsM: readonly Vec3[];
}

const SELECTION_COLOR = 0x1a45c4;
const WEAVE_COLOR = 0x7f4200;
// Torch guide colors follow the GradientOS weld preview convention:
// cyan = travel, blue = up, green = normal, orange = torch.
const TRAVEL_DIR_COLOR = 0x3f5669;
const UP_REF_COLOR = 0x1a45c4;
const NORMAL_DIR_COLOR = 0x2a6238;
const TORCH_DIR_COLOR = 0x7f4200;
const WELD_START_COLOR = 0x2a6238;
const WELD_STOP_COLOR = 0xa8231c;

/**
 * Weld start/stop markers derived from the same motion
 * runs the path lines use, so decoration always matches the drawn path.
 */
export function createPathDecorationGroup(
  points: readonly MotionPathPoint[],
  labels: readonly PathLabel[],
  palette: OverlayPalette = DEFAULT_PATH_PALETTE,
  opacity = 1,
): THREE.Group {
  const group = new THREE.Group();
  group.name = "workstation-path-decorations";
  const faded = Math.max(0, Math.min(1, opacity));
  const runs = buildMotionRuns(points);

  for (const [runIndex, run] of runs.entries()) {
    if (run.motion !== "weld" || run.pointsM.length < 2) continue;
    group.add(
      endpointMarker(`weld-start-${runIndex}`, run.pointsM[0]!, WELD_START_COLOR, faded),
      endpointMarker(`weld-stop-${runIndex}`, run.pointsM.at(-1)!, WELD_STOP_COLOR, faded),
    );
  }

  for (const label of labels) {
    const sprite = labelSprite(label, palette);
    if (sprite) group.add(sprite);
  }
  return group;
}

/**
 * Selected spreadsheet row emphasis: a bright target marker,
 * an optional torch-angle guide, and an optional
 * weave ribbon along the active weld run.
 */
export function createSelectionOverlayGroup(
  input: SelectionOverlayInput | null,
  palette: OverlayPalette = DEFAULT_PATH_PALETTE,
): THREE.Group {
  const group = new THREE.Group();
  group.name = "workstation-selection";
  if (!input) return group;

  const accent = input.motion === "weld" ? palette.weld : palette.travel;
  const marker = new THREE.Mesh(
    new THREE.SphereGeometry(0.014, 16, 12),
    new THREE.MeshBasicMaterial({
      color: SELECTION_COLOR,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
    }),
  );
  marker.position.set(...input.targetM);
  marker.renderOrder = 108;
  marker.name = "selection-target";
  group.add(marker);

  const halo = new THREE.Mesh(
    new THREE.SphereGeometry(0.022, 16, 12),
    new THREE.MeshBasicMaterial({
      color: accent,
      transparent: true,
      opacity: 0.35,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
    }),
  );
  halo.position.set(...input.targetM);
  halo.renderOrder = 107;
  group.add(halo);

  if (input.runPointsM.length >= 2) {
    const run = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(
        input.runPointsM.map((point) => new THREE.Vector3(...point)),
      ),
      new THREE.LineBasicMaterial({
        color: accent,
        depthTest: false,
        depthWrite: false,
        toneMapped: false,
      }),
    );
    run.renderOrder = 105;
    run.name = "selection-run";
    group.add(run);
  }

  if (input.torchGuide) {
    group.add(createTorchGuideGroup(input.torchGuide));
  }

  if (input.weavePointsM.length >= 2) {
    const weave = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(
        input.weavePointsM.map((point) => new THREE.Vector3(...point)),
      ),
      new THREE.LineBasicMaterial({
        color: WEAVE_COLOR,
        depthTest: false,
        depthWrite: false,
        toneMapped: false,
      }),
    );
    weave.renderOrder = 105;
    weave.name = "selection-weave-preview";
    group.add(weave);
  }
  return group;
}

export function createTorchGuideGroup(guide: TorchGuideOverlay): THREE.Group {
  const group = new THREE.Group();
  group.name = "workstation-torch-guide";
  const origin = new THREE.Vector3(...guide.originM);
  group.add(
    guideArrow(origin, guide.travelDir, 0.16, TRAVEL_DIR_COLOR, "torch-travel"),
    guideArrow(origin, [0, 0, 1], 0.11, UP_REF_COLOR, "torch-up-reference"),
    guideArrow(origin, guide.sideDir, 0.11, NORMAL_DIR_COLOR, "torch-normal"),
    guideArrow(origin, guide.torchUpDir, 0.2, TORCH_DIR_COLOR, "torch-axis"),
  );
  return group;
}

function guideArrow(
  origin: THREE.Vector3,
  direction: Vec3,
  lengthM: number,
  color: number,
  name: string,
): THREE.ArrowHelper {
  const dir = new THREE.Vector3(...direction);
  if (dir.lengthSq() < 1e-9) dir.set(0, 0, 1);
  const arrow = new THREE.ArrowHelper(
    dir.normalize(),
    origin,
    lengthM,
    color,
    lengthM * 0.24,
    lengthM * 0.12,
  );
  arrow.name = name;
  const materials = [arrow.line.material, arrow.cone.material].flat();
  for (const material of materials) {
    material.depthTest = false;
    material.depthWrite = false;
    if ("toneMapped" in material) material.toneMapped = false;
  }
  arrow.line.renderOrder = 109;
  arrow.cone.renderOrder = 109;
  return arrow;
}

function endpointMarker(
  name: string,
  positionM: Vec3,
  color: number,
  opacity = 1,
): THREE.Mesh {
  const marker = new THREE.Mesh(
    new THREE.SphereGeometry(0.0105, 12, 10),
    new THREE.MeshBasicMaterial({
      color,
      transparent: opacity < 1,
      opacity,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
    }),
  );
  marker.name = name;
  marker.position.set(...positionM);
  marker.renderOrder = 103;
  return marker;
}

function labelSprite(
  label: PathLabel,
  palette: OverlayPalette,
): THREE.Sprite | null {
  const canvas = document.createElement("canvas");
  const context = canvas.getContext("2d");
  if (!context) return null;
  const scale = 3;
  canvas.width = 46 * scale;
  canvas.height = 22 * scale;
  context.scale(scale, scale);
  context.fillStyle = label.emphasis ? "rgba(20, 26, 32, 0.96)" : "rgba(9, 11, 14, 0.82)";
  context.fillRect(0, 0, 46, 22);
  context.strokeStyle = hexColor(colorForMotion(label.motion, palette));
  context.lineWidth = label.emphasis ? 2.5 : 1.25;
  context.strokeRect(1, 1, 44, 20);
  context.fillStyle = "#edf1f5";
  context.font = "700 11px 'JetBrains Mono', Consolas, monospace";
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillText(label.text.slice(0, 6), 23, 11.5);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(
    new THREE.SpriteMaterial({
      map: texture,
      // Constant screen size: labels must not balloon as the camera zooms in.
      sizeAttenuation: false,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
      transparent: true,
    }),
  );
  sprite.name = `path-label-${safeName(label.id)}`;
  sprite.position.set(
    label.positionM[0],
    label.positionM[1],
    label.positionM[2] + 0.02,
  );
  // With sizeAttenuation off, scale is in view units at distance 1 — roughly
  // scaleY * 1.18 of the viewport height for the 46 deg orbit camera.
  const emphasis = label.emphasis ? 1.25 : 1;
  sprite.scale.set(0.046 * emphasis, 0.022 * emphasis, 1);
  sprite.renderOrder = 111;
  return sprite;
}

function hexColor(value: number): string {
  return `#${value.toString(16).padStart(6, "0")}`;
}

export function applyTcpPose(object: THREE.Object3D, pose: TcpPose): void {
  object.position.set(pose[0], pose[1], pose[2]);
  // Motion Server plans URDF fixed-axis RPY: Rz(yaw) · Ry(pitch) · Rx(roll).
  // Empirically, Three.js reconstructs that composition from the same
  // [roll,pitch,yaw] components with Euler order "ZYX", not "XYZ".
  object.rotation.set(pose[3], pose[4], pose[5], "ZYX");
  object.updateMatrix();
}

function colorForMotion(motion: MotionClass, palette: OverlayPalette): number {
  return motion === "weld" ? palette.weld : palette.travel;
}

function safeName(value: string): string {
  return value.replace(/[^a-zA-Z0-9_-]+/g, "-");
}

function disableMaterialDepth(material: THREE.Material | THREE.Material[]): void {
  const materials = Array.isArray(material) ? material : [material];
  materials.forEach((entry) => {
    entry.depthTest = false;
    entry.depthWrite = false;
  });
}
