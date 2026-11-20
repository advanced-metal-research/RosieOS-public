export interface RectLike {
  left: number;
  top: number;
  right: number;
  bottom: number;
  width: number;
  height: number;
}

export interface CssViewport {
  x: number;
  y: number;
  width: number;
  height: number;
  aspect: number;
}

export interface BufferScissor {
  x: number;
  y: number;
  width: number;
  height: number;
}

export interface PaneLayoutValidation {
  ok: boolean;
  issues: string[];
}

export interface RendererSize {
  width: number;
  height: number;
  pixelRatio: number;
}

export function rendererSize(
  width: number,
  height: number,
  devicePixelRatio: number,
  pixelRatioCap: number,
): RendererSize {
  const finiteWidth = Number.isFinite(width) ? width : 1;
  const finiteHeight = Number.isFinite(height) ? height : 1;
  const finiteRatio = Number.isFinite(devicePixelRatio) ? devicePixelRatio : 1;
  const finiteCap = Number.isFinite(pixelRatioCap) ? pixelRatioCap : 1;
  return {
    width: Math.max(1, Math.ceil(finiteWidth)),
    height: Math.max(1, Math.ceil(finiteHeight)),
    pixelRatio: Math.max(1, Math.min(finiteRatio, Math.max(1, finiteCap))),
  };
}

/**
 * Three.js setViewport/setScissor take CSS pixels and apply renderer DPR.
 * This function deliberately does not multiply by devicePixelRatio.
 */
export function viewportFromRects(canvas: RectLike, pane: RectLike): CssViewport {
  const width = Math.max(1, pane.width);
  const height = Math.max(1, pane.height);
  return {
    x: pane.left - canvas.left,
    y: canvas.bottom - pane.bottom,
    width,
    height,
    aspect: width / height,
  };
}

/** Expected driver scissor after Three.js applies renderer pixel ratio once. */
export function expectedBufferScissor(
  viewport: CssViewport,
  pixelRatio: number,
): BufferScissor {
  const ratio = Math.max(0.01, Number.isFinite(pixelRatio) ? pixelRatio : 1);
  return {
    x: Math.round(viewport.x * ratio),
    y: Math.round(viewport.y * ratio),
    width: Math.round(viewport.width * ratio),
    height: Math.round(viewport.height * ratio),
  };
}

export function validateScissorPaneLayout(
  stage: RectLike,
  canvas: RectLike,
  orbit: RectLike,
  j6: RectLike,
  tcp: RectLike,
  tolerancePx = 0.75,
): PaneLayoutValidation {
  const issues: string[] = [];
  const cameraLeft = Math.min(j6.left, tcp.left);
  if (!near(orbit.right, cameraLeft, tolerancePx)) {
    issues.push("orbit right edge does not meet camera column left edge");
  }
  if (!near(j6.bottom, tcp.top, tolerancePx)) {
    issues.push("J6 bottom edge does not meet TCP top edge");
  }
  if (!near(j6.left, tcp.left, tolerancePx) || !near(j6.right, tcp.right, tolerancePx)) {
    issues.push("J6 and TCP panes do not share a camera column");
  }
  if (![orbit, j6, tcp].every((pane) => contains(stage, pane, tolerancePx))) {
    issues.push("one or more panes fall outside the stage");
  }
  if (!covers(canvas, stage, tolerancePx)) {
    issues.push("canvas does not cover the stage");
  }
  return { ok: issues.length === 0, issues };
}

function near(left: number, right: number, tolerance: number): boolean {
  return Math.abs(left - right) <= tolerance;
}

function contains(outer: RectLike, inner: RectLike, tolerance: number): boolean {
  return inner.left >= outer.left - tolerance &&
    inner.top >= outer.top - tolerance &&
    inner.right <= outer.right + tolerance &&
    inner.bottom <= outer.bottom + tolerance;
}

function covers(outer: RectLike, inner: RectLike, tolerance: number): boolean {
  return outer.left <= inner.left + tolerance &&
    outer.top <= inner.top + tolerance &&
    outer.right >= inner.right - tolerance &&
    outer.bottom >= inner.bottom - tolerance;
}
