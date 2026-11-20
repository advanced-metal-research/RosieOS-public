import assert from "node:assert/strict";
import test from "node:test";

import { JOG_END_FRESH_MS, JOG_END_NOTE_MS, JOG_END_TOAST_MS, JogEndTracker, jogEndDetails, normalizeJogEnd } from "./jogEnd.ts";

const ME = "tab-0123456789abcdef";
const NOW = Date.parse("2026-10-10T22:40:00Z");

// The server's last_jog_end for a Cartesian hold native stopped at the end of
// its samples (internal/denseexec/jog_end.go).
function underrun(overrides: Record<string, unknown> = {}) {
  return {
    seq: 4, code: "path_underrun", cause: "path_buffer_underrun",
    text: "Jog stopped: the path was not computed fast enough to keep up (2 samples ahead at the end). Pull the trigger again; lower the speed if it repeats.",
    detail: "cause=path_buffer_underrun: native stopped at the end of the samples received native_stop=path_buffer_underrun governing=buffer_end samples_ahead=2 underruns=1",
    repull: "yes", popup: true, disarmed: false, kind: "cartesian", axis: "+X", frame: "base", client_id: ME,
    client_label: "Browser (Chrome on macOS) on http://127.0.0.1:5189", revision: 31, generation: 7,
    at: "2026-10-10T22:39:59.8Z", age_ms: 200, duration_ms: 1840, speed_unit: "mm/s", requested_speed: 400,
    achieved_speed: 396.2, distance_unit: "mm", distance: 512.3, governing: "buffer_end", samples_ahead: 2,
    last_intent_age_ms: 31, lifetime_ms: 250, ...overrides,
  };
}

test("normalizes the server's end, from the status body or alone", () => {
  const end = normalizeJogEnd({ backend: "rt_core", last_jog_end: underrun() })!;
  assert.equal(end.code, "path_underrun");
  assert.equal(end.samplesAhead, 2);
  assert.equal(end.clientId, ME);
  assert.deepEqual(normalizeJogEnd(underrun()), end);
  assert.equal(normalizeJogEnd({ backend: "rt_core" }), null);
  assert.equal(normalizeJogEnd({ last_jog_end: { code: "", seq: 1 } }), null);
  // An unknown code still shows, with its code, when the server sent no text.
  assert.equal(normalizeJogEnd({ seq: 1, code: "something_new", popup: true })!.text, "Jog stopped (something_new).");
});

test("the owning client gets the toast, once, and it times out", () => {
  const tracker = new JogEndTracker(ME);
  assert.equal(tracker.observe(underrun(), NOW), true);
  const notice = tracker.notice(NOW);
  assert.equal(notice.active, true);
  assert.equal(notice.tone, "toast");
  assert.equal(notice.kicker, "JOG STOPPED");
  assert.match(notice.text, /not computed fast enough/);
  assert.equal(notice.expanded, false);
  // The same end on the next poll is not news.
  assert.equal(tracker.observe(underrun(), NOW + 250), false);
  assert.equal(tracker.notice(NOW + JOG_END_TOAST_MS - 1).active, true);
  assert.equal(tracker.notice(NOW + JOG_END_TOAST_MS).active, false);
  assert.equal(tracker.observe(underrun({ age_ms: 600 }), NOW + JOG_END_TOAST_MS + 400), false);
});

test("another client gets a quiet note naming whose jog it was", () => {
  const tracker = new JogEndTracker("deck-00112233aabbccdd");
  tracker.observe(underrun(), NOW);
  const notice = tracker.notice(NOW);
  assert.equal(notice.tone, "note");
  assert.equal(notice.kicker, "JOG NOTE");
  assert.match(notice.text, /^Browser \(Chrome on macOS\) on http:\/\/127\.0\.0\.1:5189: Jog stopped/);
  assert.equal(tracker.notice(NOW + JOG_END_NOTE_MS).active, false);
  // A client without an id never owns an end.
  const anonymous = new JogEndTracker("");
  anonymous.observe(underrun({ client_id: "" }), NOW);
  assert.equal(anonymous.notice(NOW).tone, "note");
});

test("no pop-up for a release, the owner's own Stop, or an end already old", () => {
  const tracker = new JogEndTracker(ME);
  assert.equal(tracker.observe(underrun({ seq: 5, code: "released", cause: "release", text: "Jog released.", popup: false }), NOW), false);
  assert.equal(tracker.observe(underrun({ seq: 6, code: "halted", cause: "operator_stop", text: "Jog stopped: Stop was pressed.", popup: false, disarmed: true }), NOW), false);
  assert.equal(tracker.notice(NOW).active, false);
  // Seen first on a reload long after it happened.
  const reloaded = new JogEndTracker(ME);
  assert.equal(reloaded.observe(underrun({ age_ms: JOG_END_FRESH_MS + 1 }), NOW), false);
  assert.equal(reloaded.notice(NOW).active, false);
  // Another client's Stop does pop up for the owner, with the disarm.
  assert.equal(tracker.observe(underrun({ seq: 7, code: "halted", cause: "operator_stop", popup: true, disarmed: true, repull: "after_rearm",
    text: "Jog stopped: rosie-pendant-v5 steamdeck-pendant pressed Stop. Arm before jogging again.", ended_by: "rosie-pendant-v5 steamdeck-pendant" }), NOW), true);
  const notice = tracker.notice(NOW);
  assert.equal(notice.kicker, "JOG STOPPED · DISARMED");
  assert.equal(notice.details[0], "The robot is disarmed: arm it before jogging.");
});

test("Details opens the technical detail and holds the toast; × dismisses", () => {
  const tracker = new JogEndTracker(ME);
  tracker.observe(underrun(), NOW);
  tracker.toggleDetails(NOW + 1000);
  const open = tracker.notice(NOW + JOG_END_TOAST_MS + 5000);
  assert.equal(open.active, true);
  assert.equal(open.expanded, true);
  assert.deepEqual(open.details, [
    "Pulling the trigger again should work.",
    "path_underrun · cause path_buffer_underrun",
    "cause=path_buffer_underrun: native stopped at the end of the samples received native_stop=path_buffer_underrun governing=buffer_end samples_ahead=2 underruns=1",
    "cartesian +X (base) · asked 400.0 mm/s, reached 396.2 mm/s · travelled 512.3 mm in 1840 ms",
    "last intent 31 ms before the end (limit 250 ms) · 2 samples ahead · limited by buffer_end",
    "jog from Browser (Chrome on macOS) on http://127.0.0.1:5189",
  ]);
  // Closing the details restarts the timer.
  tracker.toggleDetails(NOW + 20000);
  assert.equal(tracker.notice(NOW + 20000 + JOG_END_TOAST_MS - 1).active, true);
  tracker.dismiss();
  assert.equal(tracker.notice(NOW + 20001).active, false);
});

test("a revised end (disarmed by the Stop fence) updates what is shown, never re-raises a dismissed one", () => {
  const tracker = new JogEndTracker(ME);
  tracker.observe(underrun(), NOW);
  const revised = underrun({ disarmed: true, repull: "after_rearm", text: "Jog stopped: the path was not computed fast enough. The robot was disarmed: arm again before jogging." });
  assert.equal(tracker.observe(revised, NOW + 300), true);
  assert.equal(tracker.notice(NOW + 300).kicker, "JOG STOPPED · DISARMED");
  tracker.dismiss();
  assert.equal(tracker.observe({ ...revised, text: revised.text + " " }, NOW + 600), false);
  assert.equal(tracker.notice(NOW + 600).active, false);
});

test("a joint jog's evidence and a stall read in their own words", () => {
  const joint = normalizeJogEnd(underrun({ kind: "joint", axis: "-J3", frame: "", speed_unit: "deg/s", requested_speed: 22.5,
    achieved_speed: 22.1, distance_unit: "deg", distance: 14.2, governing: "", samples_ahead: null }))!;
  assert.equal(jogEndDetails(joint)[3], "joint -J3 · asked 22.5 deg/s, reached 22.1 deg/s · travelled 14.2 deg in 1840 ms");
  assert.equal(jogEndDetails(joint)[4], "last intent 31 ms before the end (limit 250 ms)");
  const tracker = new JogEndTracker(ME);
  tracker.observe(underrun({ seq: 9, code: "speed_governed_to_zero", repull: "no" }), NOW);
  assert.equal(tracker.notice(NOW).kicker, "JOG STALLED");
});
