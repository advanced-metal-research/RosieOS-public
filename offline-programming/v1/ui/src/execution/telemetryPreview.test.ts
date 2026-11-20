import assert from "node:assert/strict";
import test from "node:test";
import { Window } from "happy-dom";
import { TelemetryStrip, type TelemetrySample } from "./telemetryStrip.ts";
import type { SimulationSample } from "../simulation/contracts.ts";

const browser = new Window();
Object.assign(globalThis, {
  window: browser,
  document: browser.document,
  getComputedStyle: browser.getComputedStyle.bind(browser),
});

function preview(position = 0.5): SimulationSample {
  return { modelId: "test", jointOrder: ["J1"], jointsRad: [position], source: "preview", timeS: 0, motion: "travel" };
}

function machine(position: number | null): TelemetrySample {
  return { tMs: 1000, axes: [{ position, target: position, velocity: 0.25, followingError: 0, torque: 0 }] };
}

function fixture(capture?: (seconds: number, signal: AbortSignal) => Promise<{ samples: TelemetrySample[]; note: string }>) {
  const host = document.createElement("div");
  document.body.append(host);
  const strip = new TelemetryStrip(host, { axisLabels: () => ["J1"], storage: null, capture });
  strip.setQuantity("position");
  return {
    strip, host,
    value: () => host.querySelector<HTMLElement>('[data-axis-value="0"]')?.textContent,
    previewValue: () => {
      const cell = host.querySelector<HTMLElement>('[data-axis-preview="0"]');
      return cell && !cell.hidden ? cell.textContent : "";
    },
  };
}

test("Hold freezes the live plot at the held sample while sampling continues, and Resume follows it again", () => {
  const f = fixture();
  f.strip.push({ ...machine(0.1), tMs: 1000 });
  f.strip.render();
  assert.equal(f.value(), "5.73");
  f.strip.setPaused(true);
  f.strip.push({ ...machine(0.5), tMs: 1100 });
  f.strip.render();
  assert.equal(f.value(), "5.73", "a held plot keeps showing the sample it was held at");
  assert.equal(f.strip.recorder.length, 2, "sampling continues while held");
  f.strip.setPaused(false);
  f.strip.render();
  assert.equal(f.value(), "28.65");
  f.host.remove();
});

test("preview-only and unhomed telemetry show labelled preview angles without inventing machine positions", () => {
  const f = fixture();
  f.strip.previewSample(preview());
  f.strip.render();
  assert.equal(f.value(), "—");
  assert.equal(f.previewValue(), "preview 28.65°");
  assert.equal(f.strip.recorder.length, 0);

  f.strip.push(machine(null));
  f.strip.render();
  assert.equal(f.value(), "—");
  assert.equal(f.previewValue(), "preview 28.65°");
  assert.equal(f.strip.recorder.window(10_000)[0]?.axes[0]?.position, null);
  f.host.remove();
});

test("machine and preview angles remain distinct, and stopping preview clears its readout", () => {
  const f = fixture();
  f.strip.push(machine(-0.25));
  f.strip.previewSample(preview());
  f.strip.render();
  assert.equal(f.value(), "-14.32");
  assert.equal(f.previewValue(), "preview 28.65°");

  f.strip.previewSample(null);
  f.strip.render();
  assert.equal(f.value(), "-14.32");
  assert.equal(f.previewValue(), "");
  assert.equal(f.strip.recorder.length, 1);
  f.host.remove();
});

test("preview angles are hidden for other quantities, captures, and hidden axes, then restored in live Position", () => {
  const f = fixture();
  f.strip.push(machine(-0.25));
  f.strip.previewSample(preview());
  for (const quantity of ["velocity", "acceleration", "following_error", "torque"] as const) {
    f.strip.setQuantity(quantity);
    f.strip.render();
    assert.equal(f.previewValue(), "", `${quantity} must not show a position as its value`);
  }
  f.strip.setQuantity("position");
  f.strip.showCapture([machine(0.1)], "test capture");
  f.strip.render();
  assert.equal(f.value(), "5.73");
  assert.equal(f.previewValue(), "", "capture positions use a different origin and must not mix with live preview");
  f.strip.showLive();
  f.strip.render();
  assert.equal(f.previewValue(), "preview 28.65°");
  f.strip.toggleAxis(0);
  f.strip.render();
  assert.equal(f.previewValue(), "");
  f.strip.toggleAxis(0);
  f.strip.render();
  assert.equal(f.previewValue(), "preview 28.65°");
  f.host.remove();
});

test("preview-only hover reports the cursor-nearest preview angle and removes it when stopped", t => {
  t.mock.method(performance, "now", () => 10_000);
  const f = fixture();
  const canvas = f.host.querySelector<HTMLCanvasElement>("canvas")!;
  Object.defineProperty(canvas, "offsetParent", { configurable: true, value: f.host });
  canvas.getBoundingClientRect = () => ({ x: 0, y: 0, left: 0, top: 0, width: 600, height: 200, right: 600, bottom: 200, toJSON() {} });
  // Drawing is incidental to the regression; retain real DOM pointer events
  // and tooltip rendering while accepting the canvas drawing calls.
  const context = new Proxy({}, { get: () => () => {}, set: () => true });
  canvas.getContext = (() => context) as unknown as typeof canvas.getContext;
  f.strip.previewSample(preview(1));
  f.strip.preview.previewSample(null, ["J1"], 0);
  f.strip.preview.previewSample(preview(0.5), ["J1"], 5000);
  f.strip.preview.previewSample(preview(1), ["J1"], 10_000);
  canvas.dispatchEvent(new browser.PointerEvent("pointermove", { clientX: 300, clientY: 80 }) as unknown as Event);
  f.strip.render();
  const tooltip = f.host.querySelector<HTMLElement>(".telemetry-tooltip")!;
  assert.equal(tooltip.hidden, false);
  assert.match(tooltip.textContent ?? "", /preview/i);
  assert.match(tooltip.textContent ?? "", /28\.65/);
  assert.doesNotMatch(tooltip.textContent ?? "", /57\.30/);
  f.strip.previewSample(null);
  f.strip.render();
  assert.equal(tooltip.hidden, true);
  f.host.remove();
});

test("a pending capture cannot leave preview angles showing under a different quantity or after preview stops", async () => {
  let finish!: (result: { samples: TelemetrySample[]; note: string }) => void;
  const f = fixture(() => new Promise(resolve => { finish = resolve; }));
  f.strip.previewSample(preview());
  f.strip.render();
  const pending = f.strip.capture(3);
  try {
    assert.equal(f.strip.capturing, true);
    f.strip.setQuantity("velocity");
    f.strip.render();
    assert.equal(f.previewValue(), "");
    f.strip.setQuantity("position");
    f.strip.previewSample(null);
    f.strip.render();
    assert.equal(f.previewValue(), "");
    assert.match(f.host.querySelector("[data-telemetry-meta]")?.textContent ?? "", /reading/);
  } finally {
    finish({ samples: [], note: "empty capture" });
    await pending;
    f.host.remove();
  }
});
