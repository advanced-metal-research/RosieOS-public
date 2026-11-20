import assert from "node:assert/strict";
import test from "node:test";

import { pointerEndCause, refusalEndCause, windowEndCause } from "./jogEndCause.ts";
import { JointJogError, JointJogHold, type JointJogAPI } from "./jointJog.ts";
import { cartesianJogAPI } from "./cartesianJog.ts";

test("each release path names itself", () => {
  assert.equal(pointerEndCause("pointerup"), "pointer_up");
  assert.equal(pointerEndCause("pointerleave"), "pointer_leave");
  assert.equal(pointerEndCause("pointercancel"), "pointer_cancel");
  assert.equal(pointerEndCause("lostpointercapture"), "pointer_capture_lost");
  assert.equal(windowEndCause("blur"), "focus_lost");
  assert.equal(windowEndCause("pagehide"), "page_hidden");
  assert.equal(windowEndCause("visibilitychange"), "visibility_hidden");
  assert.equal(refusalEndCause(new JointJogError("jog_input_deadline_expired", "late")), "intent_refused:jog_input_deadline_expired");
  assert.equal(refusalEndCause(new TypeError("Illegal invocation")), "intent_refused:client_error");
});

function recordingAPI(sent: Array<{ action: string; intent: Record<string, unknown> }>): JointJogAPI {
  return {
    state: async () => ({ active: false, revision: 4, generation: 0, sequence: 0, axis: 0, reason: "", cadence_ms: 50, pipelined_intents: true } as never),
    send: async (action, intent) => { sent.push({ action, intent: intent as unknown as Record<string, unknown> }); return { active: action !== "end", revision: 4 } as never; },
  };
}
const manualTimers = { every: () => 1 as unknown as ReturnType<typeof setInterval>, clear: () => undefined };

test("End carries the cause; no cause is the old release body", async () => {
  const sent: Array<{ action: string; intent: Record<string, unknown> }> = [];
  const hold = new JointJogHold(recordingAPI(sent), () => undefined, manualTimers as never);
  await hold.begin("bench", "ds-1", 1, 1, .5);
  await hold.end("pointer_leave");
  assert.deepEqual(sent.map(s => s.action), ["begin", "end"]);
  assert.equal(sent[0]!.intent.end_cause, undefined);
  assert.equal(sent[1]!.intent.end_cause, "pointer_leave");
  await hold.begin("bench", "ds-1", 1, 1, .5);
  await hold.end();
  assert.equal("end_cause" in sent[3]!.intent, false);
});

test("the Cartesian halt body carries end_cause", async () => {
  const bodies: Array<{ url: string; body: Record<string, unknown> }> = [];
  const fetcher = (async (url: string, init?: RequestInit) => {
    bodies.push({ url, body: init?.body ? JSON.parse(String(init.body)) : {} });
    return new Response(JSON.stringify({ active: false, revision: 2 }), { status: 200 });
  }) as typeof fetch;
  const api = cartesianJogAPI(() => "base", fetcher, () => ({ distanceMM: 10, angleDegrees: 5 }));
  await api.send("end", { target_id: "bench", session_id: "ds-1", revision: 2, axis: 0, direction: 1, fraction: .5, end_cause: "stick_direction_change" });
  assert.match(bodies[0]!.url, /\/cartesian\/halt$/);
  assert.equal(bodies[0]!.body.end_cause, "stick_direction_change");
  await api.send("update", { target_id: "bench", session_id: "ds-1", revision: 2, axis: 0, direction: 1, fraction: .5 });
  assert.equal("end_cause" in bodies[1]!.body, false);
});
