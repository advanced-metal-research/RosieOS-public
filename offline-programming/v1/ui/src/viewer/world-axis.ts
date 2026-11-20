import * as THREE from "three";
import { themeToken } from "../ui/theme.ts";

export type WorldAxisName = "X" | "Y" | "Z";

export interface ProjectedWorldAxis {
  name: WorldAxisName;
  color: string;
  screenDirection: readonly [number, number];
  cameraDepth: number;
}

const AXES = [
  { name: "X", color: "#ff5a5f", direction: new THREE.Vector3(1, 0, 0) },
  { name: "Y", color: "#55d98a", direction: new THREE.Vector3(0, 1, 0) },
  { name: "Z", color: "#4ba3ff", direction: new THREE.Vector3(0, 0, 1) },
] as const;

/**
 * Projects the positive world axes into camera-relative screen directions.
 * Screen X points right and screen Y points down.
 */
export function projectWorldAxes(
  cameraWorldQuaternion: THREE.Quaternion,
): readonly ProjectedWorldAxis[] {
  const worldToCamera = cameraWorldQuaternion.clone().invert();
  return AXES.map((axis) => {
    const cameraDirection = axis.direction.clone().applyQuaternion(worldToCamera);
    return {
      name: axis.name,
      color: axis.color,
      screenDirection: [cameraDirection.x, -cameraDirection.y] as const,
      cameraDepth: cameraDirection.z,
    };
  });
}

export class WorldAxisIndicator {
  private readonly canvas: HTMLCanvasElement;
  private readonly context: CanvasRenderingContext2D | null;
  private readonly cameraQuaternion = new THREE.Quaternion();
  private readonly lastCameraQuaternion = new THREE.Quaternion(
    Number.NaN,
    Number.NaN,
    Number.NaN,
    Number.NaN,
  );
  private lastWidth = 0;
  private lastHeight = 0;
  private lastPixelRatio = 0;

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;
    this.context = canvas.getContext("2d");
  }

  /** Redraw on the next frame even if the camera has not moved: the
   *  label takes its ink from the theme. */
  invalidate(): void {
    this.lastWidth = 0;
  }

  draw(camera: THREE.Camera): void {
    if (!this.context) return;
    camera.getWorldQuaternion(this.cameraQuaternion);
    const width = Math.max(1, Math.round(this.canvas.clientWidth || 92));
    const height = Math.max(1, Math.round(this.canvas.clientHeight || 92));
    const pixelRatio = Math.min(window.devicePixelRatio || 1, 2);
    if (
      width === this.lastWidth &&
      height === this.lastHeight &&
      pixelRatio === this.lastPixelRatio &&
      this.cameraQuaternion.equals(this.lastCameraQuaternion)
    ) {
      return;
    }

    this.lastWidth = width;
    this.lastHeight = height;
    this.lastPixelRatio = pixelRatio;
    this.lastCameraQuaternion.copy(this.cameraQuaternion);
    if (
      this.canvas.width !== Math.round(width * pixelRatio) ||
      this.canvas.height !== Math.round(height * pixelRatio)
    ) {
      this.canvas.width = Math.round(width * pixelRatio);
      this.canvas.height = Math.round(height * pixelRatio);
    }

    const context = this.context;
    context.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
    context.clearRect(0, 0, width, height);

    // The kicker preset, in the tertiary ink the pane labels use.
    context.fillStyle = themeToken("--fg-faint", "#8f979e");
    context.font = '600 9px "IBM Plex Mono", "SFMono-Regular", Consolas, monospace';
    context.textAlign = "left";
    context.textBaseline = "middle";
    context.fillText("WORLD XYZ", 8, 10);

    const originX = width / 2;
    const originY = height / 2 + 9;
    const axisLength = Math.min(width, height) * 0.31;
    const projected = [...projectWorldAxes(this.cameraQuaternion)].sort(
      (left, right) => right.cameraDepth - left.cameraDepth,
    );

    for (const axis of projected) {
      const endX = originX + axis.screenDirection[0] * axisLength;
      const endY = originY + axis.screenDirection[1] * axisLength;
      drawAxis(context, originX, originY, endX, endY, axis);
    }

    context.beginPath();
    context.arc(originX, originY, 2.25, 0, Math.PI * 2);
    context.fillStyle = "#dce5ec";
    context.fill();
    context.lineWidth = 1;
    context.strokeStyle = "rgba(5, 7, 10, 0.9)";
    context.stroke();
  }
}

function drawAxis(
  context: CanvasRenderingContext2D,
  originX: number,
  originY: number,
  endX: number,
  endY: number,
  axis: ProjectedWorldAxis,
): void {
  const projectedLength = Math.hypot(endX - originX, endY - originY);
  context.globalAlpha = 0.58 + Math.max(-0.2, Math.min(0.2, -axis.cameraDepth * 0.2));
  context.beginPath();
  context.moveTo(originX, originY);
  context.lineTo(endX, endY);
  context.lineWidth = 2;
  context.lineCap = "round";
  context.strokeStyle = axis.color;
  context.stroke();
  context.globalAlpha = 1;

  const radius = 7;
  context.beginPath();
  context.arc(endX, endY, radius, 0, Math.PI * 2);
  context.fillStyle = projectedLength < 2
    ? "rgba(5, 7, 10, 0.96)"
    : "rgba(11, 15, 20, 0.92)";
  context.fill();
  context.lineWidth = 1.5;
  context.strokeStyle = axis.color;
  context.stroke();

  context.fillStyle = axis.color;
  context.font = '800 8px "SFMono-Regular", Consolas, monospace';
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillText(axis.name, endX, endY + 0.5);
}
