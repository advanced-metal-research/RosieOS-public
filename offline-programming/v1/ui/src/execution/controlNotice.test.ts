import assert from "node:assert/strict";
import test from "node:test";
import { CONTROL_REFUSAL, ControlTracker, controlNoticeOf, normalizeControl } from "./controlNotice.ts";

const NOW = Date.parse("2026-10-07T22:05:00Z");

// The server's control view (control.go) while a Steam Deck holds the robot armed.
const deckArmed = {
  schema: "offline-programming.control.v1", now: "2026-10-07T22:05:00Z", state: "other",
  you: { kind: "browser", label: "Browser (Chrome on macOS) on http://127.0.0.1:5189" }, named: true,
  owner: { kind: "pendant", label: "rosie-pendant-v5 (steamdeck-pendant)", address: "100.64.0.15", loopback: false,
    since: "2026-10-07T22:01:00Z", last_action: "jog/begin", last_action_at: "2026-10-07T22:04:56Z", idle_ms: 4000 },
  armed: true, lease_held: true, take_control: false,
  take_control_note: "It is armed: control is never taken from an armed controller. Ask its operator to disarm, or press Stop (which disarms it and releases control).",
  stop_available: true, detail: "The robot is in use by rosie-pendant-v5 (steamdeck-pendant) ...",
};

test("another client armed: who, what, since, Stop, and no Take control", () => {
  const notice = controlNoticeOf(normalizeControl(deckArmed), NOW);
  assert.equal(notice.active, true);
  assert.equal(notice.expanded, true);
  assert.equal(notice.kicker, "ROBOT IN USE · ARMED");
  assert.equal(notice.who, "rosie-pendant-v5 (steamdeck-pendant) · from 100.64.0.15");
  assert.equal(notice.what, "ARMED · last request jog/begin 4 s ago");
  assert.match(notice.since, /^since 4 min ago \(\d\d:\d\d:\d\d\)$/);
  assert.equal(notice.canTake, false);
  assert.equal(notice.stopAvailable, true);
  assert.match(notice.note, /never taken from an armed controller/);
});

test("Take control is offered only when the server said it would accept it", () => {
  const idle = { ...deckArmed, armed: false, take_control: true, take_control_note: "It is disarmed and has been idle for 45s.",
    owner: { ...deckArmed.owner, last_action: "home" } };
  const notice = controlNoticeOf(normalizeControl(idle), NOW);
  assert.equal(notice.canTake, true);
  assert.equal(notice.kicker, "ROBOT IN USE");
  assert.match(notice.what, /holds the control session \(disarmed\) · last request home/);
  assert.equal(controlNoticeOf(normalizeControl({ ...idle, take_control: false }), NOW).canTake, false);
});

test("a grant held by another server: no Stop from here, the hardware stop named", () => {
  const notice = controlNoticeOf(normalizeControl({
    state: "other_server", now: "2026-10-07T22:05:00Z", stop_available: false, take_control: false,
    servo_holder: { controller: "offline-programming:server-deck01", remote: true, identity: "steamdeck-pendant-olp", since: "2026-10-07T21:58:00Z" },
  }), NOW);
  assert.equal(notice.active, true);
  assert.equal(notice.kicker, "ROBOT IN USE · ANOTHER SERVER");
  assert.equal(notice.who, "offline-programming:server-deck01 · remote steamdeck-pendant-olp");
  assert.equal(notice.stopAvailable, false);
  assert.match(notice.stopNote, /hardware stop/);
  assert.equal(notice.canTake, false);
});

test("free, yours, and an older server's answer show nothing", () => {
  for (const view of [{ state: "free" }, { state: "yours", owner: deckArmed.owner }, {}, null, { state: "bogus" }]) {
    assert.equal(controlNoticeOf(normalizeControl(view), NOW).active, false, JSON.stringify(view));
  }
});

test("Wait collapses until the holder changes; a refusal re-expands; free clears it", () => {
  const tracker = new ControlTracker();
  tracker.observe({ active: false, control: deckArmed }, NOW);
  assert.equal(tracker.notice(NOW).expanded, true);
  tracker.wait(NOW);
  const waited = tracker.notice(NOW);
  assert.equal(waited.active, true);
  assert.equal(waited.expanded, false);
  assert.equal(waited.summary, "Robot in use by rosie-pendant-v5 (steamdeck-pendant) · ARMED");
  tracker.observe({ control: deckArmed }, NOW + 2000);
  assert.equal(tracker.notice(NOW + 2000).expanded, false, "the same situation stays collapsed across polls");
  assert.equal(tracker.refused({ error: CONTROL_REFUSAL, detail: "…", control: deckArmed }, NOW + 3000), true);
  assert.equal(tracker.notice(NOW + 3000).expanded, true, "trying to act shows it again");
  tracker.wait(NOW + 3000);
  tracker.observe({ control: { ...deckArmed, owner: { ...deckArmed.owner, label: "Browser (Edge on Windows)", since: "2026-10-07T22:06:00Z" } } }, NOW + 4000);
  assert.equal(tracker.notice(NOW + 4000).expanded, true, "a new holder is a new situation");
  tracker.observe({ control: { state: "free", now: "2026-10-07T22:07:00Z" } }, NOW + 5000);
  assert.equal(tracker.notice(NOW + 5000).active, false);
  assert.equal(tracker.refused({ error: "maintenance" }, NOW), false);
});

test("times are read on the server's clock", () => {
  const tracker = new ControlTracker();
  // The client's clock is 10 min behind the server's.
  tracker.observe({ control: deckArmed }, NOW - 600_000);
  assert.match(tracker.notice(NOW - 600_000).since, /^since 4 min ago/);
});
