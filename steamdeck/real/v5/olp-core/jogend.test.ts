import assert from "node:assert/strict";
import test from "node:test";
import * as jogend from "./jogend.ts";

const DECK = "deck-00112233aabbccdd";
const T0 = Date.parse("2026-10-10T22:40:00Z");
const expired = {
  seq: 12, code: "input_deadline_expired", cause: "jog_input_deadline_expired", repull: "yes", popup: true, disarmed: false,
  text: "Jog stopped: the jog commands arrived late (last one 263 ms ago, limit 250 ms). Pull the trigger again; if it repeats, check the pendant's network link.",
  detail: "cause=jog_input_deadline_expired last_intent_age_ms=263 lifetime_ms=250: no admitted operator intent renewed the deadman in time",
  kind: "joint", axis: "+J2", client_id: DECK, client_label: "rosie-pendant-v5 deck-a", at: "2026-10-10T22:39:59.95Z", age_ms: 50,
  speed_unit: "deg/s", requested_speed: 18, achieved_speed: 17.6, distance_unit: "deg", distance: 6.1, duration_ms: 420,
  last_intent_age_ms: 263, lifetime_ms: 250,
};

test("this pendant's drop-out is a toast from the status poll; a release is not", () => {
  jogend.jogEndForget();
  assert.equal(jogend.jogEndView(T0).active, false);
  assert.equal(jogend.jogEndObserve({ backend: "rt_core", last_jog_end: expired }, T0, DECK), true);
  const view = jogend.jogEndView(T0);
  assert.equal(view.active, true);
  assert.equal(view.tone, "toast");
  assert.equal(view.kicker, "JOG STOPPED");
  assert.match(view.text, /arrived late \(last one 263 ms ago, limit 250 ms\)/);
  assert.equal(jogend.jogEndObserve({ last_jog_end: expired }, T0 + 200, DECK), false);
  assert.equal(jogend.jogEndObserve({ last_jog_end: { ...expired, seq: 13, code: "released", popup: false } }, T0 + 300, DECK), false);
});

test("DETAILS holds it open; the dismiss clears it; another client's is a note", () => {
  jogend.jogEndForget();
  jogend.jogEndObserve({ last_jog_end: expired }, T0, DECK);
  jogend.jogEndToggleDetails(T0 + 100);
  const open = jogend.jogEndView(T0 + 60000);
  assert.equal(open.expanded, true);
  assert.equal(open.details[0], "Pulling the trigger again should work.");
  jogend.jogEndDismiss();
  assert.equal(jogend.jogEndView(T0 + 60000).active, false);
  jogend.jogEndForget();
  jogend.jogEndObserve({ last_jog_end: { ...expired, client_id: "tab-0123456789abcdef", client_label: "Browser" } }, T0, DECK);
  assert.equal(jogend.jogEndView(T0).tone, "note");
});


test("an older server's direction-change hint uses the L2 controls", () => {
  jogend.jogEndForget();
  jogend.jogEndObserve({ last_jog_end: { ...expired, code: "stick_direction_change", repull: "after_neutral",
    text: "Jog stopped. Release RT and centre the stick to jog a new direction.",
  } }, T0, DECK);
  const notice = jogend.jogEndView(T0);
  assert.equal(notice.text, "Jog stopped. Centre the sticks before jogging a new direction; keep L2 held to stay armed.");
  assert.equal(notice.code, "stick_direction_change");
});
