import assert from "node:assert/strict";
import test from "node:test";
import { PRESS_NOTICE_MS, PressNoticeTracker } from "./pressNotice.ts";
import { finishRefusal, normalizeRunResult, runNotSent, runRefusal } from "./runProgram.ts";

const T0 = Date.parse("2026-10-12T07:22:17Z");
// steamdeck-pendant on the second 1430 cell, 00:22:17 PDT: the guard's 409 has no `run` member.
const NOT_FOCUSED = "rosie-pendant-v5 (steamdeck-pendant) is in the background (its window does not have input focus): a hidden pendant never arms or moves the robot. Bring it to the front to run-program. Stop always works.";

test("a refused Run 5 times names the press, says the server's sentence and that nothing moved", () => {
  const result = { ...normalizeRunResult({ error: "client_not_focused", detail: NOT_FOCUSED }), code: "client_not_focused" };
  const refusal = runRefusal(result, { mode: "count", count: 5 });
  assert.equal(refusal.kicker, "RUN 5 TIMES REFUSED");
  assert.equal(refusal.text, NOT_FOCUSED);
  assert.deepEqual(refusal.details, ["Nothing was armed, moved or played.", "client_not_focused"]);
  assert.equal(refusal.code, "client_not_focused");
});

test("a refusal after steps cleared says what happened before it", () => {
  const result = normalizeRunResult({ error: "fault_persists", run: {
    state: "refused", steps: [{ step: "arm", outcome: "done" }, { step: "move", outcome: "done" }],
    refusal: { step: "load", code: "plan_mismatch", detail: "The plan changed since it was confirmed" },
    loop: { mode: "loop", count: 0 } } });
  const refusal = runRefusal(result, { mode: "once", count: 1 });
  assert.equal(refusal.kicker, "LOOP UNTIL STOPPED REFUSED"); // the server's loop, not the stale local choice
  assert.equal(refusal.text, "armed · moved to the program's start; then The plan changed since it was confirmed");
  assert.deepEqual(refusal.details, ["Before the refusal: armed · moved to the program's start.", "plan_mismatch at step load"]);
});

test("unsent presses and Finish cycle refusals have their own words", () => {
  assert.equal(runNotSent("LOAD is still in progress", { mode: "once", count: 1 }).kicker, "RUN PROGRAM NOT SENT");
  const finish = finishRefusal("loop_owner_only", "Only steamdeck-operator may finish it");
  assert.equal(finish.kicker, "FINISH CYCLE REFUSED");
  assert.equal(finish.text, "Only steamdeck-operator may finish it");
  assert.equal(finish.details[0], "The loop keeps running. STOP ends it at once.");
});

test("the toast shows the newest refusal, goes by itself, and Details holds it open", () => {
  const tracker = new PressNoticeTracker();
  assert.equal(tracker.notice(T0).active, false);
  tracker.show(runNotSent("first", undefined), T0);
  tracker.show(finishRefusal("loop_owner_only", "second"), T0 + 10);
  const shown = tracker.notice(T0 + 10);
  assert.equal(shown.active, true);
  assert.equal(shown.text, "second");
  assert.equal(shown.tone, "toast");
  assert.equal(tracker.notice(T0 + 10 + PRESS_NOTICE_MS - 1).active, true);
  assert.equal(tracker.notice(T0 + 10 + PRESS_NOTICE_MS).active, false);
  tracker.show(finishRefusal("x", "held"), T0);
  tracker.toggleDetails(T0 + 1);
  assert.equal(tracker.notice(T0 + 10 * PRESS_NOTICE_MS).expanded, true);
  tracker.dismiss();
  assert.equal(tracker.notice(T0).active, false);
});
