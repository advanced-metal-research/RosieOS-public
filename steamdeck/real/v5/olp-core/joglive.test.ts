import assert from "node:assert/strict";
import test from "node:test";
import { jogLiveFromStatus } from "./joglive.ts";

test("the jog page's live line from a status poll during a crawl, nothing when clear and stopped", () => {
  const live = {
    active: true, axis: "-X", frame: "base", speed_unit: "mm/s", requested_speed: 400, speed: 21, governing: "cap",
    limit_kind: "singularity", limit: "singularity", gate_scale: .22, min_gate_scale_ahead: .04, near: true, j5_deg: -88.3,
    wrist_singular_deg: -90, wrist_distance_deg: 1.7, distance_unit: "mm", distance_to_end: 3.2, escape: "+X", joint_hint: "+J5",
    text: "limited by: singularity (J5 -88.3°, 1.7° from the wrist-singular -90.0°) — slowing, stops in ~3.2 mm; back out with +X, or joint-jog +J5",
  };
  const view = jogLiveFromStatus({ backend: "rt_core", jog_live: live });
  assert.equal(view.visible, true);
  assert.equal(view.speed, "21 / 400 mm/s");
  assert.equal(view.meter.label, "J5 1.7° from -90°");
  assert.equal(view.tone, "caution");
  assert.equal(jogLiveFromStatus({ backend: "rt_core", jog_live: { ...live, active: false, near: false } }).visible, false);
  assert.equal(jogLiveFromStatus({ backend: "rt_core" }).visible, false);
});
