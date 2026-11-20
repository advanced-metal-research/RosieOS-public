// @ts-nocheck — verbatim port from weld_planner/v1/ui/src; that tree
// typechecks it under its own strict tsconfig (and scalar-function is
// pinned against python/weld_programmer/scalar.py by tests). Checking
// it again under this tree's noUncheckedIndexedAccess would force a
// fork full of `!`s that could silently drift from the original.
// Ported from weld_planner/v1/ui/src/panes/prepare/spline-editor.ts —
// the profile editor rails mode shows for work and travel angle.
/**
 * A profile editor: one angle against position along the seam.
 *
 *     +60 ┤
 *         │      ●───────●
 *       0 ┤ ●────╯        ╲────●
 *     -60 ┤
 *         └────────────────────── s
 *
 * Drag a point to change the angle there. Click the track to add one, and
 * alt-click or right-click a point to remove it. The two ends are pinned to
 * s=0 and s=1 -- the schema requires a profile to span the whole seam -- so
 * they move only in value.
 *
 * Linear between points. A smooth spline through the same points overshoots
 * between them, and an overshoot in a work angle is the torch leaning
 * somewhere nobody asked for. The schema has `bspline` if that is ever wanted,
 * and it should be an explicit choice.
 *
 * A flat profile is written back as a `constant`, not as two identical knots:
 * "this does not vary" is worth saying in the document.
 */

import { el } from "./dom";
import type { Point, ScalarFunction } from "./scalar-function";
import {
  addPoint,
  evaluate,
  fromPoints,
  isEditable,
  movePoint,
  removePoint,
  toPoints,
} from "./scalar-function";

export interface SplineEditorOptions {
  label: string;
  unit: string;
  min: number;
  max: number;
  /** Drawn as a dashed line, e.g. 0 for an angle. */
  zero?: number;
  onChange(fn: ScalarFunction): void;
}

export interface SplineEditor {
  root: HTMLElement;
  set(fn: ScalarFunction | undefined, origin: string): void;
  /** Draw a marker at the previewed position, so the profile and the torch
   *  in the viewport are visibly the same thing. */
  setCursor(s: number | null): void;
  /**
   * The admissible band, drawn BEHIND the profile.
   *
   * A limit and the profile it constrains belong in one picture. Shown as two
   * numbers in a field list, a band is a fact you have to hold in your head
   * while dragging a curve somewhere else; shown as a corridor, staying inside
   * it is just aiming.
   *
   * Either side may be absent, which means unbounded on that side rather than
   * zero -- so the shading runs to the axis and the operator can see that
   * nothing is stopping them going further.
   */
  setCorridor(min: ScalarFunction | undefined, max: ScalarFunction | undefined): void;
}

// Tracks the stylesheet: `axis` is `--edge`, `text` is `--muted`. Literals for
// the same reason as cross-section.ts -- this redraws while a point is dragged.
const CSS = {
  axis: "#434b5b",
  line: "#ff5a3c",
  point: "#ffd7cc",
  text: "#8b95a3",
  cursor: "#62c2a8",
  // The admissible band. Low alpha because it is a backdrop -- the profile is
  // the subject and must stay the brightest thing in the plot.
  band: "rgba(127, 168, 255, 0.10)",
  bandEdge: "#41527a",
  // Where the profile leaves the band. Not the same red as a collision: this
  // is a document that will not validate, not a torch hitting metal.
  over: "#ff4d4d",
};

// How finely the corridor is sampled across the plot. The limits are scalar
// functions and may be piecewise, so the band is drawn as a polygon rather
// than a rectangle; 96 columns is under a pixel of error at any panel width
// this dialog reaches.
const BAND_STEPS = 96;

export function createSplineEditor(options: SplineEditorOptions): SplineEditor {
  const canvas = el("canvas", { class: "profile" });
  const readout = el("span", { class: "muted profilevalue" });
  const kindNote = el("span", { class: "muted profilekind" });
  const root = el(
    "div",
    { class: "profileblock" },
    el(
      "div",
      { class: "profilehead" },
      el("label", {}, options.label),
      readout,
      kindNote,
    ),
    canvas,
  );

  let points: Point[] = [];
  let editable = true;
  let cursor: number | null = null;
  let dragging: number | null = null;
  let lowFn: ScalarFunction | undefined;
  let highFn: ScalarFunction | undefined;

  const pad = { left: 30, right: 8, top: 8, bottom: 14 };

  const toX = (s: number, w: number) => pad.left + s * (w - pad.left - pad.right);
  const toY = (value: number, h: number) => {
    const t = (value - options.min) / (options.max - options.min);
    return h - pad.bottom - t * (h - pad.top - pad.bottom);
  };
  const fromX = (x: number, w: number) => (x - pad.left) / (w - pad.left - pad.right);
  const fromY = (y: number, h: number) =>
    options.min +
    ((h - pad.bottom - y) / (h - pad.top - pad.bottom)) * (options.max - options.min);

  function draw() {
    // A canvas inside a closed <dialog> has no layout, so clientWidth is 0.
    // Drawing anyway at a fallback size produced a small bitmap that CSS then
    // stretched to the real width when the dialog opened -- fuzzy text,
    // oversized points, wrong proportions. Skip the draw and let the
    // ResizeObserver below fire it once the element actually has a box.
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (!w || !h) return;
    const ratio = Math.min(devicePixelRatio || 1, 2);
    canvas.width = Math.round(w * ratio);
    canvas.height = Math.round(h * ratio);
    const g = canvas.getContext("2d");
    if (!g) return;
    g.setTransform(ratio, 0, 0, ratio, 0, 0);
    g.clearRect(0, 0, w, h);

    g.font = "9px ui-monospace, monospace";
    g.fillStyle = CSS.text;
    g.strokeStyle = CSS.axis;
    g.lineWidth = 1;

    // frame + the two limits, labelled, so a value is readable without a
    // tooltip
    g.beginPath();
    g.moveTo(pad.left, pad.top);
    g.lineTo(pad.left, h - pad.bottom);
    g.lineTo(w - pad.right, h - pad.bottom);
    g.stroke();
    g.textAlign = "right";
    g.fillText(String(options.max), pad.left - 4, pad.top + 7);
    g.fillText(String(options.min), pad.left - 4, h - pad.bottom);
    g.textAlign = "left";
    g.fillText("0", pad.left, h - 3);
    g.textAlign = "right";
    g.fillText("s 1", w - pad.right, h - 3);

    if (options.zero !== undefined && options.zero > options.min && options.zero < options.max) {
      const y = toY(options.zero, h);
      g.save();
      g.setLineDash([2, 3]);
      g.beginPath();
      g.moveTo(pad.left, y);
      g.lineTo(w - pad.right, y);
      g.stroke();
      g.restore();
    }

    // The admissible band, behind everything else it constrains.
    if (lowFn || highFn) {
      const xs: number[] = [];
      const los: number[] = [];
      const his: number[] = [];
      for (let i = 0; i <= BAND_STEPS; i += 1) {
        const sAt = i / BAND_STEPS;
        xs.push(toX(sAt, w));
        // Absent means UNBOUNDED on that side, so the shading runs to the axis
        // -- a band drawn to zero would say the opposite of what is declared.
        los.push(toY(lowFn ? evaluate(lowFn, sAt) : options.min, h));
        his.push(toY(highFn ? evaluate(highFn, sAt) : options.max, h));
      }
      g.save();
      g.beginPath();
      g.moveTo(xs[0], his[0]);
      for (let i = 1; i <= BAND_STEPS; i += 1) g.lineTo(xs[i], his[i]);
      for (let i = BAND_STEPS; i >= 0; i -= 1) g.lineTo(xs[i], los[i]);
      g.closePath();
      g.fillStyle = CSS.band;
      g.fill();
      g.strokeStyle = CSS.bandEdge;
      g.lineWidth = 1;
      for (const edge of [lowFn ? los : null, highFn ? his : null]) {
        if (!edge) continue;
        g.beginPath();
        g.moveTo(xs[0], edge[0]);
        for (let i = 1; i <= BAND_STEPS; i += 1) g.lineTo(xs[i], edge[i]);
        g.stroke();
      }
      g.restore();
    }

    if (!points.length) return;

    g.strokeStyle = CSS.line;
    g.lineWidth = 1.5;
    g.beginPath();
    points.forEach((point, index) => {
      const x = toX(point.s, w);
      const y = toY(point.value, h);
      if (index === 0) g.moveTo(x, y);
      else g.lineTo(x, y);
    });
    g.stroke();

    // Where the profile leaves its own declared band. Drawn rather than left
    // to the validator: this is the state in which `/api/welds/optimise`
    // refuses outright, and finding that out on a button press is worse than
    // seeing it while dragging.
    if (lowFn || highFn) {
      const outside = (sAt: number) => {
        const v = evaluate(fromPoints(points), sAt);
        return (lowFn && v < evaluate(lowFn, sAt) - 1e-9) ||
          (highFn && v > evaluate(highFn, sAt) + 1e-9);
      };
      g.save();
      g.strokeStyle = CSS.over;
      g.lineWidth = 2.5;
      let run: number[] | null = null;
      for (let i = 0; i <= BAND_STEPS; i += 1) {
        const sAt = i / BAND_STEPS;
        if (outside(sAt)) {
          const y = toY(evaluate(fromPoints(points), sAt), h);
          if (!run) {
            run = [];
            g.beginPath();
            g.moveTo(toX(sAt, w), y);
          } else {
            g.lineTo(toX(sAt, w), y);
          }
        } else if (run) {
          g.stroke();
          run = null;
        }
      }
      if (run) g.stroke();
      g.restore();
    }

    if (cursor !== null) {
      const x = toX(cursor, w);
      g.strokeStyle = CSS.cursor;
      g.lineWidth = 1;
      g.beginPath();
      g.moveTo(x, pad.top);
      g.lineTo(x, h - pad.bottom);
      g.stroke();
    }

    g.fillStyle = editable ? CSS.point : CSS.text;
    for (const point of points) {
      g.beginPath();
      g.arc(toX(point.s, w), toY(point.value, h), 3, 0, Math.PI * 2);
      g.fill();
    }
  }

  const GRAB_PX = 11;

  function nearest(x: number, y: number, w: number, h: number): number | null {
    let best: number | null = null;
    let bestDistance = GRAB_PX;
    points.forEach((point, index) => {
      const distance = Math.hypot(toX(point.s, w) - x, toY(point.value, h) - y);
      if (distance < bestDistance) {
        bestDistance = distance;
        best = index;
      }
    });
    return best;
  }

  function commit() {
    draw();
    options.onChange(fromPoints(points));
    updateReadout();
  }

  function updateReadout() {
    const fn = fromPoints(points);
    readout.textContent =
      fn.kind === "constant"
        ? `${fn.value.toFixed(1)}${options.unit}`
        : `${Math.min(...points.map((p) => p.value)).toFixed(1)} to ` +
          `${Math.max(...points.map((p) => p.value)).toFixed(1)}${options.unit}`;
  }

  // Fires when the dialog opens and whenever the pane is resized, which is the
  // only reliable moment to learn the canvas's real size.
  new ResizeObserver(() => draw()).observe(canvas);

  canvas.addEventListener("pointerdown", (event: PointerEvent) => {
    if (!editable) return;
    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    const index = nearest(x, y, rect.width, rect.height);

    // alt-click or right-click removes; the ends are not removable, because a
    // profile has to span the whole seam.
    if (index !== null && (event.altKey || event.button === 2)) {
      event.preventDefault();
      points = removePoint(points, index);
      commit();
      return;
    }
    if (event.button !== 0) return;

    if (index === null) {
      // Refuse a point that would land closer than the grab radius to one that
      // already exists: neither could then be picked up, and two stacked dots
      // read as a rendering fault rather than as two knots.
      const s = fromX(x, rect.width);
      const tooClose = points.some(
        (p) => Math.abs(toX(p.s, rect.width) - toX(s, rect.width)) < GRAB_PX,
      );
      if (tooClose) return;
      points = addPoint(points, { s, value: fromY(y, rect.height) });
      dragging = points.findIndex((p) => Math.abs(toX(p.s, rect.width) - x) < GRAB_PX);
    } else {
      dragging = index;
    }
    canvas.setPointerCapture(event.pointerId);
    commit();
  });

  canvas.addEventListener("pointermove", (event: PointerEvent) => {
    if (dragging === null) return;
    const rect = canvas.getBoundingClientRect();
    const value = Math.min(
      options.max,
      Math.max(options.min, fromY(event.clientY - rect.top, rect.height)),
    );
    points = movePoint(points, dragging, {
      s: fromX(event.clientX - rect.left, rect.width),
      value: Math.round(value * 10) / 10,
    });
    commit();
  });

  const stop = (event: PointerEvent) => {
    if (dragging === null) return;
    dragging = null;
    if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
  };
  canvas.addEventListener("pointerup", stop);
  canvas.addEventListener("pointercancel", stop);
  canvas.addEventListener("contextmenu", (event: Event) => event.preventDefault());

  return {
    root,
    set(fn, origin) {
      editable = isEditable(fn);
      points = toPoints(fn);
      canvas.dataset.editable = String(editable);
      kindNote.textContent = editable
        ? origin
        : `${fn?.kind ?? "?"} · not editable here`;
      updateReadout();
      draw();
    },
    setCursor(s) {
      cursor = s;
      draw();
    },
    setCorridor(min, max) {
      lowFn = min;
      highFn = max;
      draw();
    },
  };
}

/** The value a profile takes at `s`, for the preview. */
export const valueAt = (fn: ScalarFunction | undefined, s: number, fallback = 0): number =>
  fn ? evaluate(fn, s) : fallback;
