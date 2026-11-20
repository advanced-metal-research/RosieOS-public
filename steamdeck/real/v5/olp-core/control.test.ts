import assert from "node:assert/strict";
import test from "node:test";
import * as control from "./control.ts";

const T0 = Date.parse("2026-10-07T22:05:00Z");
const browserArmed = {
  state: "other", now: "2026-10-07T22:05:00Z", armed: true, lease_held: true, take_control: false, stop_available: true,
  take_control_note: "It is armed: control is never taken from an armed controller.",
  owner: { kind: "browser", label: "Browser (Chrome on macOS) on http://127.0.0.1:5189", address: "127.0.0.1", loopback: true,
    since: "2026-10-07T22:01:00Z", last_action: "arm", last_action_at: "2026-10-07T22:04:50Z" },
};

test("the pendant names the other controller from the poll and clears when free", () => {
  control.controlForget();
  assert.equal(control.controlView(T0).active, false);
  control.controlObserve({ active: false, control: browserArmed }, T0);
  const view = control.controlView(T0);
  assert.equal(view.active, true);
  assert.equal(view.kicker, "ROBOT IN USE · ARMED");
  assert.equal(view.who, "Browser (Chrome on macOS) on http://127.0.0.1:5189 · from 127.0.0.1 (this host or a tunnel)");
  assert.equal(view.canTake, false);
  assert.equal(view.stopAvailable, true);
  control.controlObserve({ control: { state: "free" } }, T0 + 2000);
  assert.equal(control.controlView(T0 + 2000).active, false);
});

test("WAIT collapses, a refusal re-expands, and an older server shows nothing", () => {
  control.controlForget();
  control.controlObserve({ control: browserArmed }, T0);
  control.controlWait(T0);
  assert.equal(control.controlView(T0).expanded, false);
  assert.equal(control.controlRefused({ error: "in_use_by_other_controller", control: browserArmed }, T0 + 1000), true);
  assert.equal(control.controlView(T0 + 1000).expanded, true);
  assert.equal(control.controlRefused({ error: "target_changed" }, T0), false);
  control.controlObserve({ active: false, clients: [] }, T0 + 3000);
  assert.equal(control.controlView(T0 + 3000).active, false);
});
