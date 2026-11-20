import assert from "node:assert/strict";
import test from "node:test";
import { Window } from "happy-dom";

const browser = new Window();
Object.assign(globalThis, { window: browser, document: browser.document, HTMLElement: browser.HTMLElement, Node: browser.Node });

const { JogLiveReadout } = await import("./jog-live.ts");
const { jogLiveView, normalizeJogLive } = await import("../execution/jogLive.ts");
// As execution/jogLive.test.ts: the 1430 cell's 2026-10-10 22:25 crawl.
const crawl = {
  active: true, axis: "-X", frame: "base", speed_unit: "mm/s", requested_speed: 400, speed: 21, governing: "cap",
  limit_kind: "singularity", limit: "singularity", gate_scale: .22, min_gate_scale_ahead: .04, near: true, j5_deg: -88.3,
  wrist_singular_deg: -90, wrist_distance_deg: 1.7, line_end: "jacobian_gate", distance_unit: "mm", distance_to_end: 3.2,
  escape: "+X", joint_hint: "+J5",
  text: "limited by: singularity (J5 -88.3°, 1.7° from the wrist-singular -90.0°) — slowing, stops in ~3.2 mm; back out with +X, or joint-jog +J5",
};

test("the pad's live line during a crawl, then gone when clear and stopped", () => {
  document.body.replaceChildren();
  const readout = new JogLiveReadout().mount(document.body);
  assert.equal(readout.el.hidden, true);
  readout.render(jogLiveView(normalizeJogLive(crawl)));
  assert.equal(readout.el.hidden, false);
  assert.equal(readout.el.dataset.tone, "caution");
  assert.equal(readout.el.querySelector(".jog-live-kicker")!.textContent, "PATH HOLD · -X");
  assert.equal(readout.el.querySelector(".jog-live-speed")!.textContent, "21 / 400 mm/s");
  assert.match(readout.el.querySelector(".jog-live-text")!.textContent!, /limited by: singularity \(J5 -88\.3°/);
  const meter = readout.el.querySelector<HTMLElement>(".jog-live-meter")!;
  assert.equal(meter.hidden, false);
  assert.equal(meter.getAttribute("aria-valuenow"), "96");
  assert.equal(readout.el.querySelector(".jog-live-meter-label")!.textContent, "J5 1.7° from -90°");
  assert.equal(readout.el.querySelector("button"), null);
  readout.render(jogLiveView(normalizeJogLive({ ...crawl, near: false, min_gate_scale_ahead: 1, gate_scale: 1, limit_kind: "command", text: "limited by: your speed" })));
  assert.equal(meter.hidden, true);
  assert.equal(readout.el.dataset.tone, "normal");
  readout.render(jogLiveView(normalizeJogLive({ ...crawl, active: false, near: false })));
  assert.equal(readout.el.hidden, true);
});
