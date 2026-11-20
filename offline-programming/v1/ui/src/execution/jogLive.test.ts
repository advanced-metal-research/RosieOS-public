import assert from "node:assert/strict";
import test from "node:test";

import { jogLiveView, normalizeJogLive } from "./jogLive.ts";

// The server's jog_live during the 1430 cell's 2026-10-10 22:25 crawl: −X at
// 400 mm/s, J5 at −88.3°, the resolver's scale falling, the line ending at
// its Jacobian gate ahead (internal/denseexec/path_live.go words it).
const crawl = {
  active: true, revision: 31, axis: "-X", frame: "base", speed_unit: "mm/s", requested_speed: 400, speed: 21,
  governing: "cap", limit_kind: "singularity", limit: "singularity", gate_scale: .22, min_gate_scale_ahead: .04, near: true,
  j5_deg: -88.3, wrist_singular_deg: -90, wrist_distance_deg: 1.7, line_end: "jacobian_gate", distance_unit: "mm",
  distance_to_end: 3.2, escape: "+X", joint_hint: "+J5",
  text: "limited by: singularity (J5 -88.3°, 1.7° from the wrist-singular -90.0°) — slowing, stops in ~3.2 mm; back out with +X, or joint-jog +J5",
};

test("a crawl near the wrist singularity: speed, the server's line, the meter", () => {
  const live = normalizeJogLive({ backend: "rt_core", jog_live: crawl })!;
  assert.equal(live.near, true);
  const view = jogLiveView(live);
  assert.equal(view.visible, true);
  assert.equal(view.kicker, "PATH HOLD · -X");
  assert.equal(view.speed, "21 / 400 mm/s");
  assert.ok(Math.abs(view.speedFraction - .0525) < 1e-12);
  assert.equal(view.text, crawl.text);
  assert.deepEqual(view.meter, { visible: true, closeness: .96, label: "J5 1.7° from -90°" });
  assert.equal(view.tone, "caution");
});

test("clear of the band: no meter; at the gate: danger; no geometry: the gate's own percentage", () => {
  const clear = jogLiveView(normalizeJogLive({ ...crawl, near: false, gate_scale: 1, min_gate_scale_ahead: 1, limit_kind: "command", text: "limited by: your speed" }));
  assert.equal(clear.meter.visible, false);
  assert.equal(clear.tone, "normal");
  assert.equal(clear.text, "limited by: your speed");
  const gate = jogLiveView(normalizeJogLive({ ...crawl, min_gate_scale_ahead: 0 }));
  assert.equal(gate.tone, "danger");
  const bare = jogLiveView(normalizeJogLive({ ...crawl, wrist_singular_deg: undefined, wrist_distance_deg: undefined }));
  assert.equal(bare.meter.label, "singularity gate 96 %");
});

test("after the hold: kept only when it stopped near a singularity", () => {
  const stopped = jogLiveView(normalizeJogLive({ ...crawl, active: false, distance_to_end: undefined,
    text: "stopped near a singularity (J5 -88.3°, 1.7° from the wrist-singular -90.0°); back out with +X, or joint-jog +J5" }));
  assert.equal(stopped.visible, true);
  assert.equal(stopped.kicker, "LAST STOP · -X");
  assert.equal(stopped.speed, "");
  assert.equal(stopped.meter.visible, true);
  assert.equal(jogLiveView(normalizeJogLive({ ...crawl, active: false, near: false })).visible, false);
  assert.equal(jogLiveView(normalizeJogLive({ backend: "rt_core" })).visible, false);
});
