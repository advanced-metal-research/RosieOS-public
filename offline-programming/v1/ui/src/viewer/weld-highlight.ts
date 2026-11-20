/**
 * The selected weld, drawn over everything: a wide bright line along its
 * path that blinks when the selection lands on it, so the eye finds it in a
 * busy cell, and then stays lit for as long as it is selected.
 *
 * Drawn with depth testing off -- a weld is usually inside a corner of the
 * part, and a highlight the part hides is no highlight. The caller parents
 * the root in the frame the points are in (the part's).
 */

import * as THREE from "three";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";

type V3 = [number, number, number];

/** A signal yellow, apart from every other colour a weld is drawn in
 *  (rails amber, waypoint blue, the path's own). */
const COLOR = 0xffd400;
const WIDTH_PX = 6;
/** Three blinks: on/off halves of `BLINK_MS`, then steady. */
const BLINKS = 3;
const BLINK_MS = 360;
const STEADY_OPACITY = 0.75;

export class WeldHighlight {
  readonly root = new THREE.Group();
  #line: Line2 | null = null;
  #material: LineMaterial | null = null;
  #flashStarted = 0;
  #frame: number | null = null;

  constructor() {
    this.root.name = "weld-highlight";
    this.root.renderOrder = 10;
  }

  /** Draw `points`; `flash` blinks it first. Null or fewer than two points
   *  clears it. */
  show(points: readonly V3[] | null, flash: boolean): void {
    this.#clear();
    if (!points || points.length < 2) return;
    const material = new LineMaterial({
      color: COLOR,
      linewidth: WIDTH_PX,
      worldUnits: false,
      transparent: true,
      opacity: STEADY_OPACITY,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
    });
    const line = new Line2(new LineGeometry().setPositions(points.flat()), material);
    line.computeLineDistances();
    line.renderOrder = 10;
    // Fat lines measure their width against the pane they are drawn into.
    const viewport = new THREE.Vector4();
    line.onBeforeRender = (renderer) => {
      renderer.getCurrentViewport(viewport);
      material.resolution.set(Math.max(1, viewport.z), Math.max(1, viewport.w));
      material.linewidth = WIDTH_PX * renderer.getPixelRatio();
    };
    this.root.add(line);
    this.#line = line;
    this.#material = material;
    if (flash) {
      this.#flashStarted = performance.now();
      this.#tick();
    }
  }

  hide(): void {
    this.#clear();
  }

  dispose(): void {
    this.#clear();
    this.root.removeFromParent();
  }

  #tick = (): void => {
    this.#frame = null;
    const line = this.#line;
    const material = this.#material;
    if (!line || !material) return;
    const elapsed = performance.now() - this.#flashStarted;
    if (elapsed >= BLINKS * BLINK_MS) {
      line.visible = true;
      material.opacity = STEADY_OPACITY;
      return;
    }
    const lit = (elapsed % BLINK_MS) < BLINK_MS / 2;
    line.visible = lit;
    material.opacity = 1;
    this.#frame = requestAnimationFrame(this.#tick);
  };

  #clear(): void {
    if (this.#frame !== null) cancelAnimationFrame(this.#frame);
    this.#frame = null;
    if (this.#line) {
      this.#line.geometry.dispose();
      this.#material?.dispose();
      this.#line.removeFromParent();
    }
    this.#line = null;
    this.#material = null;
  }
}
