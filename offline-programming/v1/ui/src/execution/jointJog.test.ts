import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import {
  JointJogError, JointJogHold, JointJogPanel, JointAngleMove, jointJogAPI, jointJogDisabled, jointJogFailureText,
  jointDescribedRangeRefusal, jointJogSpeedNote, browserJogTimers, JOINT_JOG_CADENCE_MS, JOINT_JOG_MAX_IN_FLIGHT, JOINT_JOG_CLIENT_ERROR, JOINT_JOG_FRACTIONS, PREVIEW_COLUMN_TITLE,
  JOINT_ZERO_TOLERANCE_DEG, type JointJogAPI,
} from "./jointJog.ts";
import { STATUS_FRESHNESS_MS, type ArmFact } from "./machineFlow.ts";
import { RECOVER_TO_ZERO_ACTION, type RecoverConfirmation, type RecoverResult } from "./recoverToZero.ts";
import type { DenseCapabilities, DenseStatus } from "./denseClient.ts";
import { jointJogBrowserStopText, jointMoveLimitsText, rtCoreReasonText } from "../targets/rtCoreStatus.ts";

function deferred<T>() {
  let resolve!: (value: T) => void;
  let reject!: (error: Error) => void;
  const promise = new Promise<T>((yes, no) => { resolve = yes; reject = no; });
  return { promise, resolve, reject };
}

function armedFact(): ArmFact {
  return { state: "armed", controlled: true, jogNote: "", headline: "ARMED", token: "ARMED", detail: "", pending: null, transportRetry: null };
}

function harness() {
  const calls: { action: string; intent: unknown }[] = [];
  const timers = new Map<ReturnType<typeof setInterval>, () => void>();
  const state = { active: false, revision: 3, reason: "", pipelined_intents: true };
  let reply: Promise<typeof state> | null = null;
  let stateReply: Promise<typeof state> | null = null;
  const api: JointJogAPI = {
    state: () => stateReply ?? Promise.resolve(state),
    send: (action, intent) => { calls.push({ action, intent }); return reply ?? Promise.resolve(state); },
  };
  const hold = new JointJogHold(api, () => {}, {
    every: (fn, ms) => {
      assert.equal(ms, 50);
      const id = {} as ReturnType<typeof setInterval>;
      timers.set(id, fn); return id;
    },
    clear: id => { timers.delete(id); },
  });
  return {
    hold, calls, timers, api, state,
    setReply: (value: Promise<typeof state> | null) => { reply = value; },
    setState: (value: Promise<typeof state> | null) => { stateReply = value; },
    tick: () => { for (const tick of timers.values()) tick(); },
  };
}

test("JointJog hold sends one intent per cadence and one End across repeated release", async () => {
  const h = harness();
  assert.equal(JOINT_JOG_CADENCE_MS, 50);
  assert.deepEqual(JOINT_JOG_FRACTIONS, [0.05, 0.1, 0.25, 0.5, 0.75, 1]);
  await h.hold.begin("bench", "authority", 0, -1, 0.1);
  assert.deepEqual(h.calls[0], { action: "begin", intent: { target_id: "bench", session_id: "authority", revision: 3, axis: 0, direction: -1, fraction: 0.1 } });
  h.tick(); await Promise.resolve();
  h.tick(); await Promise.resolve();
  await h.hold.end(); await h.hold.end(); h.tick();
  assert.deepEqual(h.calls.map(c => c.action), ["begin", "update", "update", "end"]);
  assert.equal(h.timers.size, 0);
  assert.equal(h.hold.busy, false);
});

test("JointJog release wins over pending state, Begin and Update without replay", async () => {
  for (const phase of ["state", "begin", "update"]) {
    const h = harness(), pending = deferred<typeof h.state>();
    if (phase === "state") h.setState(pending.promise);
    if (phase === "begin") h.setReply(pending.promise);
    const begin = h.hold.begin("bench", "authority", 0, 1, 0.1);
    await Promise.resolve();
    if (phase === "update") {
      // Slow intents do not stop the cadence; JOINT_JOG_MAX_IN_FLIGHT bounds them.
      await begin; h.setReply(pending.promise);
      for (let i = 0; i < JOINT_JOG_MAX_IN_FLIGHT + 2; i++) h.tick();
      assert.equal(h.calls.filter(c => c.action === "update").length, JOINT_JOG_MAX_IN_FLIGHT);
    }
    h.setReply(null);
    await h.hold.end();
    pending.resolve(h.state); await begin; await Promise.resolve();
    h.tick();
    assert.equal(h.hold.active, false);
    assert.equal(h.timers.size, 0);
    assert.deepEqual(h.calls.map(c => c.action), phase === "state" ? [] : phase === "begin" ? ["begin", "end"] : ["begin", ...Array(JOINT_JOG_MAX_IN_FLIGHT).fill("update"), "end"]);
  }
});

test("JointJog second hold is refused and adapter reasons stop held input", async () => {
  const h = harness();
  await h.hold.begin("bench", "authority", 0, 1, 0.1);
  await h.hold.begin("bench", "authority", 1, 1, 0.1);
  assert.match(h.hold.reason, /^jog_mode_conflict:/);
  assert.equal(h.calls.length, 1);
  h.hold.observeReason("jog_stale_sequence", 3);
  assert.equal(h.hold.active, true);
  h.hold.observeReason("jog_input_deadline_expired", 4);
  assert.equal(h.hold.active, false);
  assert.equal(h.calls[1]?.action, "end");
  assert.match(h.hold.reason, /^jog_input_deadline_expired:/);
});

test("JointJog HTTP preserves typed refusals and does not retry failed input", async () => {
  for (const code of ["jog_not_ready", "jog_mode_conflict", "jog_stale_sequence", "jog_input_too_old", "outside_limits_outward", "unknown_adapter_reason"]) {
    const requests: { url: string; init?: RequestInit }[] = [];
    const api = jointJogAPI((async (url: string, init?: RequestInit) => {
      requests.push({ url, init });
      if (url.endsWith("/state")) return new Response(JSON.stringify({ active: false, revision: 1, reason: "" }));
      return new Response(JSON.stringify({ error: code, detail: "adapter detail" }), { status: 409 });
    }) as typeof fetch);
    const hold = new JointJogHold(api, () => {});
    await hold.begin("bench", "s", 0, 1, 0.1);
    assert.equal(hold.active, false);
    assert.match(hold.reason, new RegExp(`^${code}:`));
    assert.deepEqual(requests.map(r => r.url.split("/").at(-1)), ["state", "begin", "end"]);
    assert.equal(requests[2]?.init?.keepalive, true);
    assert.equal(requests[1]?.init?.credentials, "same-origin");
  }
  assert.match(rtCoreReasonText("jog_limited"), /^jog_limited: Jog limited/);
});

test("JointJog a rejected request ends the hold once and shows a client error, never transport loss", async () => {
  const h = harness();
  await h.hold.begin("bench", "s", 0, 1, 0.1);
  const pending = deferred<typeof h.state>(); h.setReply(pending.promise); h.tick();
  h.setReply(null); pending.reject(new Error("offline"));
  await Promise.resolve(); await Promise.resolve();
  assert.equal(h.hold.active, false);
  assert.equal(h.hold.reason, "client error: offline");
  assert.doesNotMatch(h.hold.reason, /rt_core_transport_lost/);
  assert.deepEqual(h.calls.map(c => c.action), ["begin", "update", "end"]);
});

test("JointJog failure text keeps typed server refusals and labels browser exceptions as client errors", () => {
  // The bench defect: a DOM TypeError rendered as the server's transport code.
  assert.equal(jointJogFailureText(new TypeError("Illegal invocation")), "client error: Illegal invocation");
  assert.equal(jointJogFailureText("boom"), "client error: boom");
  assert.equal(jointJogFailureText(new JointJogError("jog_limited", "adapter detail")), rtCoreReasonText("jog_limited", "adapter detail"));
  assert.match(jointJogFailureText(new JointJogError("unknown_code", "adapter detail")), /^unknown_code: adapter detail$/);
  for (const text of [jointJogFailureText(new TypeError("Illegal invocation")), jointJogFailureText(new Error("Failed to fetch"))]) {
    assert.doesNotMatch(text, /rt_core_transport_lost/);
  }
});

test("JointJog default timers call setInterval as a free function, not as a method of the timers object", async () => {
  // Chrome throws `TypeError: Illegal invocation` when window.setInterval runs
  // with a plain object as `this`; Node ignores `this`, so this test records it.
  const realEvery = globalThis.setInterval, realClear = globalThis.clearInterval;
  const seen: unknown[] = []; const cleared: unknown[] = [];
  const token = { id: "interval" } as unknown as ReturnType<typeof setInterval>;
  Object.assign(globalThis, {
    setInterval: function (this: unknown) { seen.push(this); return token; },
    clearInterval: function (this: unknown, id: unknown) { seen.push(this); cleared.push(id); },
  });
  try {
    const timers = browserJogTimers();
    assert.equal(timers.every(() => {}, JOINT_JOG_CADENCE_MS), token);
    timers.clear(token);
    assert.deepEqual(cleared, [token]);
    for (const receiver of seen) assert.ok(receiver === undefined || receiver === globalThis, "timer called with a foreign `this`");
    // The hold's default path is the same one.
    const calls: string[] = [];
    const api: JointJogAPI = { state: async () => ({ active: false, revision: 1, reason: "" }), send: async action => { calls.push(action); return { active: true, revision: 1, reason: "" }; } };
    const hold = new JointJogHold(api, () => {});
    await hold.begin("bench", "s", 0, 1, 0.1);
    assert.equal(hold.active, true); assert.equal(hold.reason, "");
    await hold.end();
    assert.deepEqual(calls, ["begin", "end"]);
    for (const receiver of seen) assert.ok(receiver === undefined || receiver === globalThis, "hold timer called with a foreign `this`");
  } finally { Object.assign(globalThis, { setInterval: realEvery, clearInterval: realClear }); }
});

test("JointJog state poll exception under a live hold ends it with a client error", async () => {
  const h = harness();
  const oldWindow = globalThis.window, oldDocument = globalThis.document;
  Object.assign(globalThis, { window: new EventTarget(), document: { createElement: (tag: string) => new ElementStub(tag) } });
  try {
    const host = new ElementStub();
    const panel = new JointJogPanel(host as unknown as HTMLElement, h.api);
    panel.update(true, caps, status, false, armedFact(), "bench_one_motor_1to1");
    const buttons = host.all().filter(c => c.className.includes("joint-jog-hold"));
    const down = new Event("pointerdown"); Object.assign(down, { button: 0, isPrimary: true });
    buttons[0]!.dispatchEvent(down);
    await Promise.resolve(); await Promise.resolve();
    assert.equal(panel.active, true);
    h.setState(Promise.reject(new TypeError("Illegal invocation")));
    await panel.poll();
    await Promise.resolve();
    assert.equal(panel.active, false);
    assert.equal(host.querySelector("[data-jog-reason]")!.textContent, "client error: Illegal invocation");
  } finally { Object.assign(globalThis, { window: oldWindow, document: oldDocument }); }
});

test("JointMove a browser exception refuses the move as a client error with its words", async () => {
  const h = harness();
  h.api.move = () => Promise.reject(new TypeError("Illegal invocation"));
  const move = new JointAngleMove(h.api, () => {});
  await move.begin("mini-bench", "s", 0, -30, 0.25);
  assert.equal(move.state.state, "refused");
  assert.equal(move.state.reason, JOINT_JOG_CLIENT_ERROR);
  assert.equal(move.state.detail, "client error: Illegal invocation");
});

const caps = { backend: "rt_core", cell: "mini-bench", daemonReachable: true, rtCore: { axes: [{ id: "J1", position_unit: "rad" }] } } as DenseCapabilities;
const status = { armed: true, session: { id: "s", leaseHeld: true, state: "idle" }, rtCore: { ready: true, axes: [{ position: 0.25 }] } } as DenseStatus;

test("JointJog disabled states explain absent target, description, grant and readiness", () => {
  assert.match(jointJogDisabled(false, caps, status, false, armedFact()), /No machine connected/);
  assert.match(jointJogDisabled(true, null, status, false, armedFact()), /Describe/);
  assert.match(jointJogDisabled(true, caps, null, false, armedFact()), /No grant/);
  assert.match(jointJogDisabled(true, caps, { ...status, armed: false }, false, armedFact()), /ARM/);
  assert.match(jointJogDisabled(true, caps, status, true, armedFact()), /Another target operation/);
  assert.equal(jointJogDisabled(true, caps, status, false, armedFact()), "");
});

// Teaching (owner, 2026-10-12): a planned program loaded but not playing never
// disables the jog; a playing one or a running loop does, between cycles too.
test("JointJog stays enabled with a plan loaded, and is disabled while it plays or a loop runs", () => {
  const loaded = { ...status, session: { ...status.session!, state: "loaded" } } as DenseStatus;
  assert.equal(jointJogDisabled(true, caps, loaded, false, armedFact()), "");
  const playing = { ...status, session: { ...status.session!, state: "playing" } } as DenseStatus;
  assert.match(jointJogDisabled(true, caps, playing, false, armedFact()), /Another target operation/);
  for (const state of ["running", "finishing"] as const) {
    const between = { ...status, session: { ...status.session!, state: "done" }, loop: { state } } as DenseStatus;
    assert.match(jointJogDisabled(true, caps, between, false, armedFact()), /program loop is running; STOP it/);
  }
  const ended = { ...status, session: { ...status.session!, state: "done" }, loop: { state: "ended" } } as DenseStatus;
  assert.equal(jointJogDisabled(true, caps, ended, false, armedFact()), "");
});

class ElementStub extends EventTarget {
  children: ElementStub[] = [];
  dataset: Record<string, string> = {};
  attributes = new Map<string, string>();
  textContent = ""; className = ""; innerHTML = ""; value = ""; disabled = false; title = ""; type = "";
  readonly tag: string;
  constructor(tag = "div") { super(); this.tag = tag; }
  append(...children: ElementStub[]) { this.children.push(...children); }
  replaceChildren() { this.children = []; }
  setAttribute(name: string, value: string) { this.attributes.set(name, value); }
  all(): ElementStub[] { return [this, ...this.children.flatMap(c => c.all())]; }
  querySelector(selector: string) {
    return this.all().find(c => selector === "select" ? c.tag === "select" : selector === "[data-jog-caption]" ? c.dataset.jogCaption !== undefined : c.dataset.jogReason !== undefined) ?? null;
  }
}

test("JointJog actual panel handlers end on pointer release, leave, cancel and window blur", async () => {
  const oldWindow = globalThis.window, oldDocument = globalThis.document;
  const windowStub = new EventTarget();
  Object.assign(globalThis, { window: windowStub, document: { createElement: (tag: string) => new ElementStub(tag) } });
  try {
    for (const release of ["pointerup", "pointerleave", "pointercancel", "blur", "pagehide"]) {
      const host = new ElementStub(), h = harness();
      const panel = new JointJogPanel(host as unknown as HTMLElement, h.api);
      panel.update(true, caps, status, false, armedFact(), "bench_one_motor_1to1");
      const buttons = host.all().filter(c => c.className.includes("joint-jog-hold"));
      assert.equal(buttons.length, 2);
      assert.equal(host.querySelector("[data-jog-caption]")?.textContent, "Jogs the selected robot: mini-bench");
      const down = new Event("pointerdown"); Object.assign(down, { button: 0, isPrimary: true });
      buttons[0]!.dispatchEvent(down);
      await Promise.resolve(); await Promise.resolve();
      panel.update(true, caps, { ...status, rtCore: { ...status.rtCore!, axes: [{ ...status.rtCore!.axes[0]!, position: 0.5 }] } }, false, armedFact(), "bench_one_motor_1to1");
      // The status update repainted the rows in place: the same hold button instance, no re-render.
      assert.equal(host.all().filter(c => c.className.includes("joint-jog-hold"))[0], buttons[0]);
      assert.ok(host.all().some(c => c.textContent === "28.65°"));
      (release === "blur" || release === "pagehide" ? windowStub : buttons[0]!).dispatchEvent(new Event(release));
      await Promise.resolve();
      assert.deepEqual(h.calls.map(c => c.action), ["begin", "end"]);
      panel.update(false, caps, status, false, armedFact(), "bench_one_motor_1to1");
      assert.ok(buttons.every(b => b.disabled));
      assert.match(host.querySelector("[data-jog-reason]")!.textContent, /No machine connected/);
    }
  } finally { Object.assign(globalThis, { window: oldWindow, document: oldDocument }); }
});

test("JointMove sends signed degrees and fraction, showing done only after completion", async () => {
  const h = harness();
  const completion = deferred<{ axis: number; state: string; reason: string; revision: number }>();
  const requests: unknown[] = [];
  h.api.move = (intent, signal) => { requests.push({ intent, signal }); return completion.promise; };
  const move = new JointAngleMove(h.api, () => {});
  const run = move.begin("mini-bench", "s", 0, -30, 0.25);
  await Promise.resolve();
  assert.equal(move.busy, true); assert.equal(move.state.state, "moving");
  assert.deepEqual((requests[0] as { intent: unknown }).intent, { target_id: "mini-bench", session_id: "s", revision: 3, axis: 0, degrees: -30, fraction: 0.25 });
  completion.resolve({ axis: 0, state: "done", reason: "", revision: 4 }); await run;
  assert.equal(move.busy, false); assert.equal(move.state.state, "done");
});

test("JointMove actual GO row uses the selected fraction and disables holds until completion", async () => {
  const oldWindow = globalThis.window, oldDocument = globalThis.document;
  Object.assign(globalThis, { window: new EventTarget(), document: { createElement: (tag: string) => new ElementStub(tag) } });
  try {
    const h = harness(), host = new ElementStub();
    const completion = deferred<{ axis: number; state: string; reason: string; revision: number }>();
    let request: unknown;
    h.api.move = intent => { request = intent; return completion.promise; };
    const panel = new JointJogPanel(host as unknown as HTMLElement, h.api);
    panel.update(true, caps, status, false, armedFact(), "bench_one_motor_1to1");
    const select = host.all().find(c => c.tag === "select")!;
    select.value = "0.25"; select.dispatchEvent(new Event("change"));
    host.all().find(c => c.tag === "input")!.value = "30";
    const go = host.all().find(c => c.textContent === "GO")!;
    go.dispatchEvent(new Event("click")); await Promise.resolve();
    assert.deepEqual(request, { target_id: "mini-bench", session_id: "s", revision: 3, axis: 0, degrees: 30, fraction: 0.25 });
    assert.ok(host.all().filter(c => c.tag === "button").every(c => c.disabled));
    assert.ok(host.all().some(c => c.className === "joint-move-state" && c.textContent === "moving"));
    completion.resolve({ axis: 0, state: "done", reason: "", revision: 4 });
    await Promise.resolve(); await Promise.resolve();
    assert.ok(host.all().some(c => c.className === "joint-move-state" && c.textContent === "done"));
    assert.equal(go.disabled, false);
  } finally { Object.assign(globalThis, { window: oldWindow, document: oldDocument }); }
});

test("JointMove Stop cancels pending state and active requests; late completion cannot replace stopped", async () => {
  for (const phase of ["state", "move"]) {
    const h = harness(), pendingState = deferred<typeof h.state>();
    const completion = deferred<{ axis: number; state: string; reason: string; revision: number }>();
    let signal: AbortSignal | null = null;
    h.api.move = (_intent, value) => { signal = value; return completion.promise; };
    if (phase === "state") h.setState(pendingState.promise);
    const move = new JointAngleMove(h.api, () => {});
    const run = move.begin("bench", "s", 0, 30, 0.1); await Promise.resolve();
    move.cancel();
    if (phase === "move") assert.equal((signal as AbortSignal | null)?.aborted, true);
    pendingState.resolve(h.state); completion.resolve({ axis: 0, state: "done", reason: "", revision: 4 }); await run;
    assert.equal(move.state.state, "stopped"); assert.equal(move.state.reason, "operator_stop");
    if (phase === "state") assert.equal(signal, null);
  }
});

test("JointMove API refuses non-completion replies and preserves soft limit reasons", async () => {
  for (const [body, statusCode, reason] of [
    [{ state: "moving", axis: 0, revision: 4 }, 200, "rt_core_protocol_error"],
    [{ error: "joint_move_limits", detail: "endpoint outside limits" }, 409, "joint_move_limits"],
  ] as const) {
    const api = jointJogAPI((async () => new Response(JSON.stringify(body), { status: statusCode })) as typeof fetch);
    await assert.rejects(api.move!({ target_id: "bench", session_id: "s", revision: 3, axis: 0, degrees: 30, fraction: 0.5 }, new AbortController().signal), (error: unknown) => error instanceof JointJogError && error.code === reason);
  }
  const h = harness(); h.api.move = async () => { throw new JointJogError("joint_move_limits", "limit"); };
  const move = new JointAngleMove(h.api, () => {});
  await move.begin("bench", "s", 0, 300, 0.5);
  assert.equal(move.state.reason, "joint_move_limits"); assert.equal(move.state.state, "refused");
  assert.match(rtCoreReasonText(move.state.reason), /soft position limits/);
});

test("JointJog dense integration retains simulator separation and urgent Disarm", async () => {
  const dense = await readFile(new URL("./densePanel.ts", import.meta.url), "utf8");
  assert.match(dense, /new JointJogPanel/);
  assert.match(dense, /this\.#jointJog\?\.update\(this\.#targetSelected/);
  assert.match(dense, /this\.#jointJog\?\.poll\(\)/);
  assert.match(dense, /await this\.#stop\("DISARM"\)/);
  assert.match(dense, /this\.#button\("#dense-arm-off"\)\.disabled = !this\.#targetSelected;/);
  const source = await readFile(new URL("./jointJog.ts", import.meta.url), "utf8");
  assert.doesNotMatch(source, /live-jog|cartesian|simulation\//);
  assert.match(source, /Jogs the selected robot:/);
});

const J5_HOT = "J5 Er41.1 (Motor over-temperature due to locked- rotor) still active — let it cool, then press again";

test("All joints to 0° is the one-press recovery: armed it moves at the displayed speed; disarmed it asks naming the action and each joint, and only a confirmation arms", async () => {
  const h = harness();
  const bodies: Record<string, unknown>[] = [];
  const answers: RecoverResult[] = [];
  let refuse: string | null = null;
  h.api.recover = async body => {
    bodies.push(body);
    if (refuse) throw new JointJogError("fault_persists", refuse);
    return answers.shift()!;
  };
  const asked: RecoverConfirmation[] = [];
  let answer = true;
  const refusals: string[] = [];
  const oldWindow = globalThis.window, oldDocument = globalThis.document;
  Object.assign(globalThis, { window: new EventTarget(), document: { createElement: (tag: string) => new ElementStub(tag) } });
  try {
    const host = new ElementStub();
    const panel = new JointJogPanel(host as unknown as HTMLElement, h.api, text => refusals.push(text), {
      home: () => {}, homeDisabled: () => "Disarm first: Home runs with the drives off",
      confirmRecover: async confirmation => { asked.push(confirmation); return answer; },
    });
    const threeAxes = { ...caps, rtCore: { axes: [{ id: "J1", position_unit: "rad" }, { id: "J2", position_unit: "rad" }, { id: "J3", position_unit: "counts" }] } } as DenseCapabilities;
    const at = (positions: number[]) => ({ ...status, rtCore: { ...status.rtCore!, axes: positions.map(position => ({ ...status.rtCore!.axes[0]!, position })) } } as DenseStatus);
    panel.update(true, threeAxes, at([Math.PI / 6, 0.0001, 5]), false, armedFact(), "rosie_1420_v1");
    const zero = host.all().find(c => c.className.includes("joint-jog-zero"))!;
    assert.equal(zero.disabled, false);
    const speed = host.all().find(c => c.tag === "select")!;
    speed.value = "0.25"; speed.dispatchEvent(new Event("change"));
    // Armed: one press, the server moves; no question.
    answers.push({ state: "done", action: RECOVER_TO_ZERO_ACTION, steps: [{ step: "move", outcome: "done" }], joints: [], refusal: null });
    await panel.zeroAll();
    assert.deepEqual([...bodies], [{ fraction: 0.25, confirm_arm: false }]);
    assert.equal(asked.length, 0);
    assert.equal(host.querySelector("[data-jog-reason]")!.textContent, "All joints at 0° (moved to 0°)");
    assert.equal(zero.textContent, "All joints to 0°");

    // Disarmed and stuck (the the 1430 cell 12:19 state): the button stays usable,
    // the first answer asks, the confirmation carries the joints back.
    const stuck = { ...at([Math.PI / 6, 0.0001, 5]), armed: false, session: null, daemonError: "operator_stop: control_session_stale" } as DenseStatus;
    panel.update(true, threeAxes, stuck, false, { ...armedFact(), state: "disarmed", controlled: false }, "rosie_1420_v1");
    assert.equal(zero.disabled, false, "disarmed is what the recovery is for");
    const joints = [{ axis: 0, id: "J1", from_deg: 30, to_deg: 0, delta_deg: -30, moves: true }, { axis: 1, id: "J2", from_deg: 0.0057, to_deg: 0, delta_deg: -0.0057, moves: false }];
    const confirmFirst: RecoverResult = { state: "confirm_arm", action: RECOVER_TO_ZERO_ACTION, joints, refusal: null,
      steps: [{ step: "start", outcome: "done" }, { step: "confirm_stop", outcome: "done" }, { step: "reset", outcome: "done" }, { step: "arm", outcome: "needs_confirmation" }] };
    bodies.length = 0;
    answers.push(confirmFirst, { state: "done", action: RECOVER_TO_ZERO_ACTION, joints, refusal: null,
      steps: [{ step: "arm", outcome: "done" }, { step: "move", outcome: "done" }] });
    await panel.zeroAll(1);
    assert.deepEqual([...bodies], [{ fraction: 1, confirm_arm: false }, { fraction: 1, confirm_arm: true, confirmed: [{ axis: 0, id: "J1", from_deg: 30 }, { axis: 1, id: "J2", from_deg: 0.0057 }] }]);
    assert.equal(speed.value, "0.25", "a one-move speed override preserves the joint pane's choice");
    assert.equal(asked.length, 1);
    assert.equal(asked[0]!.title, "Arm and move all joints to zero");
    assert.deepEqual(asked[0]!.lines, ["J1  30.00° → 0.00°  Δ −30.00°", "J2  0.01° → 0.00°  stays"]);
    assert.match(asked[0]!.intro, /^Stop confirmed · faults reset\. The robot is disarmed/);
    assert.match(asked[0]!.warning, /straight line in joint space.*may sweep/);
    assert.match(host.querySelector("[data-jog-reason]")!.textContent, /^All joints at 0° \(armed · moved to 0°\)\. /);

    // Cancel: nothing armed or moved, and only one request.
    bodies.length = 0; answer = false; answers.push(confirmFirst);
    await panel.zeroAll();
    assert.equal(bodies.length, 1);
    assert.match(host.querySelector("[data-jog-reason]")!.textContent, /^All joints to 0° cancelled: nothing was armed or moved/);

    // J5 still hot: the server's sentence, over the jog gate, and to Logs.
    refuse = J5_HOT;
    await panel.zeroAll();
    assert.match(host.querySelector("[data-jog-reason]")!.textContent, /^J5 Er41\.1 \(Motor over-temperature due to locked- rotor\) still active — let it cool, then press again\./);
    assert.equal(refusals.at(-1), `All joints to 0°: ${J5_HOT}`);
    assert.deepEqual([...panel.zeroing], []);

    // A status poll while the press waits (its own Reset retired the session)
    // must not abort it.
    refuse = null; answer = true; bodies.length = 0;
    const held = deferred<RecoverResult>();
    h.api.recover = async body => { bodies.push(body); return bodies.length === 1 ? confirmFirst : held.promise; };
    const press = panel.zeroAll();
    await new Promise(resolve => setTimeout(resolve, 0));
    panel.update(true, threeAxes, { ...stuck, session: { id: "s2", leaseHeld: true, state: "idle" } } as DenseStatus, false, { ...armedFact(), state: "disarmed", controlled: false }, "rosie_1420_v1");
    held.resolve({ state: "done", action: RECOVER_TO_ZERO_ACTION, joints, refusal: null, steps: [{ step: "move", outcome: "done" }] });
    await press;
    assert.equal(bodies.length, 2);
    assert.match(host.querySelector("[data-jog-reason]")!.textContent, /^All joints at 0°/);
  } finally { Object.assign(globalThis, { window: oldWindow, document: oldDocument }); }
});

test("each joint row offers Home for that joint alone under the whole-arm gate", async () => {
  const h = harness();
  const homed: number[] = [];
  let gate: string | null = "Disarm first: Home runs with the drives off";
  const oldWindow = globalThis.window, oldDocument = globalThis.document;
  Object.assign(globalThis, { window: new EventTarget(), document: { createElement: (tag: string) => new ElementStub(tag) } });
  try {
    const host = new ElementStub();
    const panel = new JointJogPanel(host as unknown as HTMLElement, h.api, () => {}, { home: () => {}, homeAxis: index => { homed.push(index); }, homeDisabled: () => gate });
    const twoAxes = { ...caps, rtCore: { axes: [{ id: "J1", position_unit: "rad" }, { id: "J2", position_unit: "rad" }] } } as DenseCapabilities;
    const st = { ...status, rtCore: { ...status.rtCore!, axes: [{ position: 0.1 }, { position: 0.2 }] } } as DenseStatus;
    panel.update(true, twoAxes, st, false, armedFact(), "rosie_1420_v1");
    const buttons = host.all().filter(c => c.className.includes("joint-home-axis"));
    assert.equal(buttons.length, 2);
    assert.equal(buttons[1]!.disabled, true);
    assert.equal(buttons[1]!.title, gate);
    buttons[1]!.dispatchEvent(new Event("click"));
    assert.deepEqual(homed, [], "a gated Home does not fire");
    gate = null;
    panel.update(true, twoAxes, st, false, armedFact(), "rosie_1420_v1");
    assert.equal(buttons[1]!.disabled, false);
    buttons[1]!.dispatchEvent(new Event("click"));
    assert.deepEqual(homed, [1]);
  } finally { Object.assign(globalThis, { window: oldWindow, document: oldDocument }); }
});

test("joint Home status follows fresh per-axis evidence, never the Home button", () => {
  const h = harness(), requests: number[] = [];
  const oldWindow = globalThis.window, oldDocument = globalThis.document;
  Object.assign(globalThis, { window: new EventTarget(), document: { createElement: (tag: string) => new ElementStub(tag) } });
  try {
    const host = new ElementStub();
    const panel = new JointJogPanel(host as unknown as HTMLElement, h.api, () => {}, { homeAxis: index => requests.push(index) });
    const described = { ...caps, rtCore: { axes: [{ index: 0, id: "J1", position_unit: "rad" }, { index: 1, id: "J2", position_unit: "rad" }] } } as DenseCapabilities;
    const observed = { ...status, available: true, backend: "rt_core", rtCore: { ...status.rtCore!, operatorFeedback: { axes: [] }, axes: [{ position: 0, homeValid: true }, { position: 0, homeValid: false }] } } as unknown as DenseStatus;
    const states = () => host.all().filter(c => c.dataset.jogHomeState !== undefined);
    const labels = () => states().map(c => c.textContent);
    panel.update(true, described, observed, false, armedFact(), "rosie_1420_v1", 0);
    assert.deepEqual(labels(), ["Homed", "Home required"]);
    panel.update(true, described, observed, true, armedFact(), "rosie_1420_v1", 0, [0]);
    assert.deepEqual(labels(), ["Homing", "Home required"], "re-Home pending replaces the previous Homed label before native status arrives");
    panel.update(true, described, observed, false, armedFact(), "rosie_1420_v1", 0);
    const originalStates = states();
    host.all().filter(c => c.className.includes("joint-home-axis"))[1]!.dispatchEvent(new Event("click"));
    assert.deepEqual(requests, [1]);
    assert.deepEqual(labels(), ["Homed", "Home required"], "a sent Home request does not establish validity");

    const homing = { ...observed, rtCore: { ...observed.rtCore!, axes: [observed.rtCore!.axes[0]!, { ...observed.rtCore!.axes[1]!, homeActive: true }] } };
    panel.update(true, described, homing, false, armedFact(), "rosie_1420_v1", 0);
    assert.deepEqual(labels(), ["Homed", "Homing"]);

    panel.update(true, described, observed, false, armedFact(), "rosie_1420_v1", STATUS_FRESHNESS_MS + 1);
    assert.deepEqual(states().map(c => c.dataset.jogHomeState), ["stale", "stale"]);
    assert.ok(labels().every(label => /stale/i.test(label)));
    assert.equal(states()[0], originalStates[0], "status polling preserves row controls and their focus");

    panel.update(true, described, observed, false, armedFact(), "rosie_1420_v1", 0);
    assert.deepEqual(labels(), ["Homed", "Home required"]);
    const absentAxis = { ...observed, rtCore: { ...observed.rtCore!, axes: [observed.rtCore!.axes[0]!] } };
    panel.update(true, described, absentAxis, false, armedFact(), "rosie_1420_v1", 0);
    assert.deepEqual(labels(), ["Homed", "Status unavailable"]);
    panel.update(false, described, observed, false, armedFact(), "rosie_1420_v1", 0);
    assert.deepEqual(labels(), ["Status unavailable", "Status unavailable"]);
  } finally { Object.assign(globalThis, { window: oldWindow, document: oldDocument }); }
});

test("joint Home status remains visible without Home actions and refuses unknown freshness", () => {
  const oldWindow = globalThis.window, oldDocument = globalThis.document;
  Object.assign(globalThis, { window: new EventTarget(), document: { createElement: (tag: string) => new ElementStub(tag) } });
  try {
    const host = new ElementStub(), panel = new JointJogPanel(host as unknown as HTMLElement, harness().api);
    const observed = { ...status, available: true, backend: "rt_core", rtCore: { ...status.rtCore!, axes: [{ position: 0, homeValid: true }] } } as DenseStatus;
    panel.update(true, caps, observed, false, armedFact(), "bench_one_motor_1to1");
    const home = host.all().find(c => c.dataset.jogHomeState !== undefined)!;
    assert.ok(home, "read-only joint table still shows Home state");
    assert.notEqual(home.textContent, "Homed", "no freshness evidence must not display retained Home as current");
    panel.update(true, caps, null, false, armedFact(), "bench_one_motor_1to1", 0);
    assert.equal(home.textContent, "Status unavailable");
  } finally { Object.assign(globalThis, { window: oldWindow, document: oldDocument }); }
});

const SIM_MIN_RAD = -0.5;
const SIM_MAX_RAD = 0.5;
const J7_LIMITS_TEXT = `joint_move_limits: ${jointMoveLimitsText("J7", SIM_MIN_RAD, SIM_MAX_RAD)}`;

function simAxis(id: string) {
  return { id, position_unit: "rad" as const, min_position: SIM_MIN_RAD, max_position: SIM_MAX_RAD };
}

function sevenCaps() {
  return { ...caps, rtCore: { axes: ["J1", "J2", "J3", "J4", "J5", "J6", "J7"].map(simAxis) } } as DenseCapabilities;
}

function sevenStatus(degrees: number[]) {
  return { ...status, rtCore: { ...status.rtCore!, axes: degrees.map(value => ({ position: value * Math.PI / 180 })) } } as DenseStatus;
}

function rowStates(host: ElementStub) {
  return host.all().filter(c => c.className.includes("joint-move-state"));
}

function hungMove(h: ReturnType<typeof harness>) {
  h.api.move = (_intent, signal) => new Promise((_resolve, reject) => {
    signal.addEventListener("abort", () => reject(new DOMException(String(signal.reason), "AbortError")));
  });
}

test("joint_move_limits names the joint and described range in degrees, and 0.5 rad is ±28.6°", () => {
  assert.equal(jointMoveLimitsText("J7", -0.5, 0.5), "Angle exceeds J7's described range (-28.6° to 28.6°)");
  assert.equal(rtCoreReasonText("joint_move_limits", jointMoveLimitsText("J7", -0.5, 0.5)), J7_LIMITS_TEXT);
  assert.match(rtCoreReasonText("joint_move_limits"), /soft position limits/);
  const described = { id: "J7", position_unit: "rad", min_position: -0.5, max_position: 0.5, origin_rad: 0 };
  const refusal = jointDescribedRangeRefusal(described, 45);
  assert.ok(refusal);
  assert.equal(refusal!.code, "joint_move_limits");
  assert.equal(refusal!.message, "Angle exceeds J7's described range (-28.6° to 28.6°)");
  assert.equal(jointDescribedRangeRefusal(described, 15), null);
});

test("a later joint_move_limits on J7 replaces J1's operator_stop on that row and the pane line", async () => {
  const oldWindow = globalThis.window, oldDocument = globalThis.document;
  Object.assign(globalThis, { window: new EventTarget(), document: { createElement: (tag: string) => new ElementStub(tag) } });
  try {
    const h = harness(), host = new ElementStub();
    hungMove(h);
    const panel = new JointJogPanel(host as unknown as HTMLElement, h.api);
    panel.update(true, sevenCaps(), sevenStatus([0, 0, 0, 0, 0, 0, 0]), false, armedFact(), "rosie_1420_v1");
    host.all().filter(c => c.tag === "input")[0]!.value = "15";
    host.all().filter(c => c.textContent === "GO")[0]!.dispatchEvent(new Event("click"));
    await Promise.resolve();
    panel.stop();
    await Promise.resolve();
    const rows = rowStates(host);
    assert.equal(rows[0]!.textContent, rtCoreReasonText("operator_stop"));
    assert.equal(host.querySelector("[data-jog-reason]")!.textContent, rtCoreReasonText("operator_stop"));
    h.api.move = async () => { throw new Error("hopeless GO reached the server"); };
    host.all().filter(c => c.tag === "input")[6]!.value = "45";
    host.all().filter(c => c.textContent === "GO")[6]!.dispatchEvent(new Event("click"));
    await Promise.resolve();
    assert.equal(rowStates(host)[6]!.textContent, J7_LIMITS_TEXT);
    assert.equal(rowStates(host)[0]!.textContent, "");
    assert.equal(host.querySelector("[data-jog-reason]")!.textContent, J7_LIMITS_TEXT);
  } finally { Object.assign(globalThis, { window: oldWindow, document: oldDocument }); }
});

test("a stop the browser sent names the event; STOP the operator pressed stays operator_stop", async () => {
  const oldWindow = globalThis.window, oldDocument = globalThis.document;
  const windowStub = new EventTarget();
  Object.assign(globalThis, { window: windowStub, document: { createElement: (tag: string) => new ElementStub(tag) } });
  try {
    const h = harness(), host = new ElementStub();
    hungMove(h);
    const panel = new JointJogPanel(host as unknown as HTMLElement, h.api);
    panel.update(true, sevenCaps(), sevenStatus([0, 0, 0, 0, 0, 0, 0]), false, armedFact(), "rosie_1420_v1");
    host.all().filter(c => c.tag === "input")[0]!.value = "15";
    host.all().filter(c => c.textContent === "GO")[0]!.dispatchEvent(new Event("click"));
    await Promise.resolve();
    windowStub.dispatchEvent(new Event("blur"));
    await Promise.resolve();
    assert.equal(rowStates(host)[0]!.textContent, jointJogBrowserStopText("page_blur"));
    assert.equal(host.querySelector("[data-jog-reason]")!.textContent, jointJogBrowserStopText("page_blur"));
    assert.doesNotMatch(rowStates(host)[0]!.textContent, /Stopped by operator/);
    hungMove(h);
    host.all().filter(c => c.tag === "input")[0]!.value = "15";
    host.all().filter(c => c.textContent === "GO")[0]!.dispatchEvent(new Event("click"));
    await Promise.resolve();
    panel.stop();
    await Promise.resolve();
    assert.equal(rowStates(host)[0]!.textContent, rtCoreReasonText("operator_stop"));
    const holdHost = new ElementStub(), holdPanel = new JointJogPanel(holdHost as unknown as HTMLElement, h.api);
    holdPanel.update(true, sevenCaps(), sevenStatus([0, 0, 0, 0, 0, 0, 0]), false, armedFact(), "rosie_1420_v1");
    const holdButton = holdHost.all().filter(c => c.className.includes("joint-jog-hold"))[0]!;
    const down = new Event("pointerdown"); Object.assign(down, { button: 0, isPrimary: true });
    holdButton.dispatchEvent(down);
    await Promise.resolve(); await Promise.resolve();
    holdButton.dispatchEvent(new Event("pointerup"));
    await Promise.resolve();
    assert.equal(rowStates(holdHost)[0]!.textContent, jointJogBrowserStopText("pointer_release"));
    assert.equal(holdHost.querySelector("[data-jog-reason]")!.textContent, jointJogBrowserStopText("pointer_release"));
  } finally { Object.assign(globalThis, { window: oldWindow, document: oldDocument }); }
});

test("All joints to 0° after a stopped step presses at the joint speed and says why when still blocked; STOP aborts a running press", async () => {
  const oldWindow = globalThis.window, oldDocument = globalThis.document;
  const windowStub = new EventTarget();
  Object.assign(globalThis, { window: windowStub, document: { createElement: (tag: string) => new ElementStub(tag) } });
  try {
    const h = harness(), host = new ElementStub();
    const sent: Record<string, unknown>[] = [];
    hungMove(h);
    const panel = new JointJogPanel(host as unknown as HTMLElement, h.api);
    panel.update(true, sevenCaps(), sevenStatus([0, 0, 0, 0, 0, 0, 0]), false, armedFact(), "rosie_1420_v1");
    host.all().filter(c => c.tag === "input")[0]!.value = "15";
    host.all().filter(c => c.textContent === "GO")[0]!.dispatchEvent(new Event("click"));
    await Promise.resolve();
    h.api.recover = async body => { sent.push(body); return { state: "done", action: RECOVER_TO_ZERO_ACTION, steps: [], joints: [], refusal: null }; };
    await panel.zeroAll();
    assert.equal(sent.length, 0);
    assert.equal(host.querySelector("[data-jog-reason]")!.textContent, "Another target motion is active");
    windowStub.dispatchEvent(new Event("blur"));
    await Promise.resolve();
    panel.update(true, sevenCaps(), sevenStatus([10.23, 0, JOINT_ZERO_TOLERANCE_DEG / 2, 0, 0, 0, 0]), false, armedFact(), "rosie_1420_v1");
    await panel.zeroAll();
    assert.deepEqual([...sent], [{ fraction: 0.1, confirm_arm: false }]);
    let signal: AbortSignal | null = null;
    h.api.recover = (_body, value) => { signal = value; return new Promise((_resolve, reject) => value.addEventListener("abort", () => reject(new DOMException("stop", "AbortError")))); };
    const press = panel.zeroAll();
    await Promise.resolve();
    panel.stop();
    await press;
    assert.equal((signal as AbortSignal | null)?.aborted, true);
    assert.match(host.querySelector("[data-jog-reason]")!.textContent, /^All joints to 0° stopped/);
  } finally { Object.assign(globalThis, { window: oldWindow, document: oldDocument }); }
});

function previewSample(jointOrder: readonly string[], jointsRad: readonly number[]) {
  return { jointOrder, jointsRad };
}

test("Preview column shows the previewed plan in degrees beside the machine Position, maps by axis id then index, and clears to — when stopped", async () => {
  const oldWindow = globalThis.window, oldDocument = globalThis.document;
  Object.assign(globalThis, { window: new EventTarget(), document: { createElement: (tag: string) => new ElementStub(tag) } });
  try {
    const h = harness();
    const host = new ElementStub();
    const panel = new JointJogPanel(host as unknown as HTMLElement, h.api);
    const matching = {
      ...caps,
      rtCore: { axes: [{ id: "J1", position_unit: "rad" }, { id: "J2", position_unit: "rad" }] },
    } as DenseCapabilities;
    const live = {
      ...status,
      rtCore: { ...status.rtCore!, axes: [{ position: 0.1 }, { position: 0.2 }] },
    } as DenseStatus;
    panel.update(true, matching, live, false, armedFact(), "rosie_1420_v1");
    const table = host.all().find(c => c.tag === "table")!;
    assert.match(table.innerHTML, /Preview/);
    assert.match(table.innerHTML, new RegExp(PREVIEW_COLUMN_TITLE.replace(/[.*+?^${}()|[\]\\]/g, "\\$&")));
    const positions = () => host.all().filter(c => c.dataset.jogPosition !== undefined);
    const previews = () => host.all().filter(c => c.dataset.jogPreview !== undefined);
    assert.equal(positions().length, 2);
    assert.equal(previews().length, 2);
    assert.equal(positions()[0]!.textContent, "5.73°");
    assert.equal(positions()[1]!.textContent, "11.46°");
    assert.equal(previews()[0]!.textContent, "—");
    assert.equal(previews()[1]!.textContent, "—");

    panel.previewSample(previewSample(["J1", "J2"], [0.5, -0.25]));
    assert.equal(previews()[0]!.textContent, "28.65°");
    assert.equal(previews()[1]!.textContent, "-14.32°");
    assert.equal(positions()[0]!.textContent, "5.73°", "Position stays the machine's 0.1 rad");
    assert.equal(positions()[1]!.textContent, "11.46°", "Position stays the machine's 0.2 rad");

    panel.previewSample(null);
    assert.equal(previews()[0]!.textContent, "—");
    assert.equal(previews()[1]!.textContent, "—");
    assert.equal(positions()[0]!.textContent, "5.73°");

    const unnamed = {
      ...caps,
      rtCore: { axes: [{ id: "axis_a", position_unit: "rad" }, { id: "axis_b", position_unit: "rad" }] },
    } as DenseCapabilities;
    panel.update(true, unnamed, live, false, armedFact(), "rosie_1420_v1");
    panel.previewSample(previewSample(["J1", "J2"], [0.5, -0.25]));
    assert.equal(previews()[0]!.textContent, "28.65°");
    assert.equal(previews()[1]!.textContent, "-14.32°");
  } finally { Object.assign(globalThis, { window: oldWindow, document: oldDocument }); }
});

test("Speed runs to 100% and the note says what the percentage is of, in degrees per second", () => {
  assert.deepEqual([...JOINT_JOG_FRACTIONS], [0.05, 0.1, 0.25, 0.5, 0.75, 1]);
  const alike = [{ position_unit: "rad", max_velocity: 0.1 }, { position_unit: "rad", max_velocity: 0.1 }];
  // 0.1 rad/s is 5.73 deg/s; 10 % of it is 0.57 deg/s.
  assert.equal(jointJogSpeedNote(0.1, alike), "of each joint's described maximum 5.73°/s: 0.57°/s");
  // At 100 % the result would repeat the maximum, so only the maximum is said.
  assert.equal(jointJogSpeedNote(1, alike), "of each joint's described maximum 5.73°/s");
  const mixed = [{ id: "J1", position_unit: "rad", max_velocity: 0.1 }, { id: "J6", position_unit: "rad", max_velocity: 0.2 }];
  assert.equal(jointJogSpeedNote(0.5, mixed), "of each joint's described maximum, 5.73°/s (J1) to 11.5°/s (J6): 2.86°/s to 5.73°/s");
  assert.equal(jointJogSpeedNote(1, mixed), "of each joint's described maximum, 5.73°/s (J1) to 11.5°/s (J6)");
  // The 3000 rpm cap through 100:1 and 10:1 gearing: 180 and 1800 deg/s, said without decimals.
  const rated = [{ id: "J1", position_unit: "rad", max_velocity: 3.14159265 }, { id: "J6", position_unit: "rad", max_velocity: 31.41592654 }];
  assert.equal(jointJogSpeedNote(0.1, rated), "of each joint's described maximum, 180°/s (J1) to 1800°/s (J6): 18.0°/s to 180°/s");
  assert.equal(jointJogSpeedNote(0.1, [{ position_unit: "m", max_velocity: 0.1 }]), "");
  assert.equal(jointJogSpeedNote(0.1, [{ position_unit: "rad" }]), "");
});

test("the Speed select stays usable while the pane is disabled and its note follows the choice", () => {
  const oldWindow = globalThis.window, oldDocument = globalThis.document;
  Object.assign(globalThis, { window: new EventTarget(), document: { createElement: (tag: string) => new ElementStub(tag) } });
  try {
    const h = harness(), host = new ElementStub();
    const panel = new JointJogPanel(host as unknown as HTMLElement, h.api);
    const described = { ...caps, rtCore: { axes: [{ id: "J1", position_unit: "rad", max_velocity: 0.1 }] } } as DenseCapabilities;
    // No status: the pane is disabled ("No grant"), the hold and GO buttons with it.
    panel.update(true, described, null, false, armedFact(), "bench_one_motor_1to1");
    const select = host.all().find(c => c.tag === "select")!;
    assert.equal(select.disabled, false, "choosing a speed is not a motion");
    assert.ok(host.all().filter(c => c.tag === "button").every(c => c.disabled));
    const note = () => host.all().find(c => c.dataset.jogSpeedNote !== undefined)!;
    assert.equal(note().textContent, "of each joint's described maximum 5.73°/s: 0.57°/s");
    select.value = "1"; select.dispatchEvent(new Event("change"));
    assert.equal(note().textContent, "of each joint's described maximum 5.73°/s");
  } finally { Object.assign(globalThis, { window: oldWindow, document: oldDocument }); }
});


test("Jog caption names the verified model instead of the native pair ID", () => {
  const oldWindow = globalThis.window, oldDocument = globalThis.document;
  Object.assign(globalThis, { window: new EventTarget(), document: { createElement: (tag: string) => new ElementStub(tag) } });
  try {
    const host = new ElementStub(), h = harness();
    const panel = new JointJogPanel(host as unknown as HTMLElement, h.api);
    const observed = { ...status, robotCell: { configured: true, valid: true, modelId: "rosie_1420_v1" } } as DenseStatus;
    const described = { ...caps, cell: "local-dev" };
    panel.update(true, described, observed, false, armedFact(), "rosie_1420_v1");
    assert.equal(host.querySelector("[data-jog-caption]")?.textContent, "Jogs the selected robot: rosie_1420_v1");
    panel.update(true, described, { ...observed, robotCell: { ...observed.robotCell!, valid: false } }, false, armedFact(), "rosie_1420_v1");
    assert.equal(host.querySelector("[data-jog-caption]")?.textContent, "Jogs the selected robot: local-dev");
    panel.update(false, described, observed, false, armedFact(), "rosie_1420_v1");
    assert.equal(host.querySelector("[data-jog-caption]")?.textContent, "Jogs the selected robot: none connected");
  } finally { Object.assign(globalThis, { window: oldWindow, document: oldDocument }); }
});

/** Virtual time: the hold's 50 ms interval and each reply's delay run on one
 *  clock, so send times are exact in absolute milliseconds. */
function slowLink(replyMs: (n: number) => number, opts: { stale?: (n: number) => boolean; legacy?: boolean } = {}) {
  let now = 0, seq = 0, inFlight = 0, maxInFlight = 0;
  const sends: { action: string; at: number; aborted: boolean }[] = [];
  const due: { at: number; settle: () => void }[] = [];
  const intervals = new Map<ReturnType<typeof setInterval>, { fn: () => void; ms: number; next: number }>();
  const state = { active: false, revision: 3, reason: "", pipelined_intents: opts.legacy !== true };
  const api: JointJogAPI = {
    state: async () => state,
    send: (action, _intent, _target, signal) => {
      const n = ++seq, record = { action, at: now, aborted: false };
      sends.push(record);
      if (action === "end") return Promise.resolve(state);
      inFlight++; maxInFlight = Math.max(maxInFlight, inFlight);
      return new Promise((resolve, reject) => {
        const done = () => { inFlight--; };
        signal?.addEventListener("abort", () => { record.aborted = true; done(); reject(new DOMException("aborted", "AbortError")); });
        due.push({ at: now + replyMs(n), settle: () => {
          if (record.aborted) return;
          done();
          if (opts.stale?.(n)) reject(new JointJogError("jog_session_stale", "not this revision yet"));
          else resolve(state);
        } });
      });
    },
  };
  const hold = new JointJogHold(api, () => {}, {
    every: (fn, ms) => { const id = {} as ReturnType<typeof setInterval>; intervals.set(id, { fn, ms, next: now + ms }); return id; },
    clear: id => { intervals.delete(id); },
  });
  const flush = async () => { for (let i = 0; i < 6; i++) await Promise.resolve(); };
  const advance = async (ms: number) => {
    for (const end = now + ms; now < end;) {
      now++;
      for (const d of due.splice(0).filter(d => { if (d.at <= now) return true; due.push(d); return false; })) d.settle();
      await flush();
      for (const t of intervals.values()) if (t.next <= now) { t.next += t.ms; t.fn(); }
      await flush();
    }
  };
  const gaps = (from = 0, to = Infinity) => {
    const at = sends.filter(s => (s.action === "begin" || s.action === "update") && s.at >= from && s.at <= to).map(s => s.at);
    return at.slice(1).map((t, i) => t - at[i]!);
  };
  return { hold, sends, advance, gaps, now: () => now, maxInFlight: () => maxInFlight };
}

test("JointJog keeps the 50 ms cadence while a slow press and slow intents are in flight", async () => {
  // A round trip up to one lifetime of cadence (5 x 50 ms) costs no gap: the
  // first intent leaves 50 ms after the press, while the press is unanswered.
  for (const rtt of [140, 200, 240]) {
    const link = slowLink(() => rtt);
    const begin = link.hold.begin("bench", "s", 0, 1, 0.1);
    await link.advance(2000);
    await begin;
    const gaps = link.gaps();
    assert.ok(gaps.length >= 38, `${rtt} ms: ${gaps.length} sends`);
    assert.ok(Math.max(...gaps) <= 60, `${rtt} ms round trips left a ${Math.max(...gaps)} ms gap`);
    assert.ok(link.maxInFlight() <= JOINT_JOG_MAX_IN_FLIGHT);
    await link.hold.end();
  }
});

test("JointJog replies of 200-400 ms stay inside the 250 ms deadman, bounded in flight", async () => {
  // Deterministic 200-400 ms replies. Gaps reach at most round trip - 4 x 50 ms
  // = 200 ms, under OLP's 250 ms input lifetime; the old single-request pacing
  // left a full round trip (up to 400 ms) between intents.
  let x = 7;
  const link = slowLink(() => { x = (x * 1103515245 + 12345) % 2147483648; return 200 + (x % 201); });
  void link.hold.begin("bench", "s", 0, 1, 0.1);
  await link.advance(3000);
  const gaps = link.gaps();
  assert.ok(Math.max(...gaps) <= 200, `gap ${Math.max(...gaps)} ms`);
  assert.equal(link.maxInFlight(), JOINT_JOG_MAX_IN_FLIGHT);
  await link.hold.end();
});

test("JointJog release stops the cadence at once, aborts in-flight intents and sends End", async () => {
  const link = slowLink(() => 300);
  void link.hold.begin("bench", "s", 0, 1, 0.1);
  await link.advance(500);
  const released = link.now();
  const end = link.hold.end();
  await link.advance(1000); await end;
  const after = link.sends.filter(s => s.at >= released);
  assert.deepEqual(after.map(s => s.action), ["end"], "nothing but End after release");
  assert.ok(link.sends.filter(s => s.action === "update" && s.aborted).length > 0, "in-flight intents were aborted");
  assert.equal(link.hold.active, false); assert.equal(link.hold.reason, "");
});

test("JointJog ignores an intent's stale refusal sent before the press was answered, not after", async () => {
  // OLP refuses intents stale while an ordinary Begin opens the lane: those
  // were sent before the press's reply and must not end the hold.
  const early = slowLink(n => (n === 1 ? 300 : 20), { stale: n => n > 1 && n <= 6 });
  void early.hold.begin("bench", "s", 0, 1, 0.1);
  await early.advance(600);
  assert.equal(early.hold.active, true, early.hold.reason);
  await early.hold.end();
  // After the press is answered a stale refusal is the hold's end.
  const late = slowLink(() => 20, { stale: n => n === 5 });
  void late.hold.begin("bench", "s", 0, 1, 0.1);
  await late.advance(600);
  assert.equal(late.hold.active, false);
  assert.match(late.hold.reason, /^jog_session_stale/);
});

test("JointJog late replies of a released hold do not touch the next hold", async () => {
  const link = slowLink(() => 300);
  void link.hold.begin("bench", "s", 0, 1, 0.1);
  await link.advance(400);
  void link.hold.end();
  void link.hold.begin("bench", "s", 1, 1, 0.1);
  await link.advance(1000);
  assert.equal(link.hold.active, true);
  assert.equal(link.hold.axis, 1);
  assert.equal(link.hold.reason, "");
  await link.hold.end();
});

test("JointJog a press while the previous release is in flight is sent, not refused", async () => {
  const calls: string[] = [];
  const endReply = deferred<{ active: boolean; revision: number; reason: string }>();
  let revision = 3;
  const api: JointJogAPI = {
    state: async () => ({ active: false, revision, reason: "" }),
    send: async action => {
      calls.push(action);
      if (action === "end") { revision++; return endReply.promise; }
      return { active: true, revision, reason: "" };
    },
  };
  const hold = new JointJogHold(api, () => {}, { every: () => ({}) as ReturnType<typeof setInterval>, clear: () => {} });
  await hold.begin("bench", "s", 0, 1, 0.1);
  const release = hold.end();
  assert.equal(hold.busy, true); assert.equal(hold.pressBusy, false);
  await hold.begin("bench", "s", 0, -1, 0.1);
  assert.deepEqual(calls, ["begin", "end", "begin"]);
  assert.equal(hold.reason, "");
  assert.equal(hold.active, true);
  endReply.resolve({ active: false, revision, reason: "" }); await release;
  await hold.end();
});

test("JointJog keeps one request in flight against a server without pipelined_intents", async () => {
  // Such a server refuses an older intent that overtook a newer one at its
  // lock (jog_invalid_timing): the old pacing stays until it reports ordering.
  const link = slowLink(() => 200, { legacy: true });
  void link.hold.begin("bench", "s", 0, 1, 0.1);
  await link.advance(2000);
  assert.equal(link.maxInFlight(), 1);
  assert.ok(Math.max(...link.gaps()) >= 200);
  await link.hold.end();
});
