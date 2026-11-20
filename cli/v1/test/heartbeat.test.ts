// The keeper must stop on its own: when its window ends, and when OLP no
// longer holds a session. Either way OLP's 5 s deadman then owns the robot.
// While one of our robot actions is in flight it must not stop for either
// reason (the action may arm and move before it answers), and no hung
// request may hold up the next beat.
import assert from "node:assert/strict";
import { once } from "node:events";
import http from "node:http";
import { test } from "node:test";

import { HeartbeatKeeper } from "../src/lib/heartbeat.js";
import { OlpClient } from "../src/lib/olp.js";

/** extend() fires its first beat without awaiting it; let it finish. */
const settle = () => new Promise((resolve) => setImmediate(resolve));
const sleep = (ms: number) => new Promise((resolve) => setTimeout(resolve, ms));

/** OLP as the keeper uses it: status, and a heartbeat refused dense_session_mismatch when no session is held (rt_core.go Heartbeat). */
function fakeOlp(held: () => boolean) {
  const beats: unknown[] = [];
  const timeouts: Array<[string, number | undefined]> = [];
  const olp = {
    status: async (timeoutMs?: number) => {
      timeouts.push(["status", timeoutMs]);
      return { available: true, armed: held(), session: held() ? { id: "s", kind: "trajectory", state: "playing", lease: { held: true, expires_in_ms: 400 } } : null };
    },
    post: async (_path: string, body: unknown, options: { timeoutMs?: number } = {}) => {
      timeouts.push(["heartbeat", options.timeoutMs]);
      if (!held()) return { status: 409, ok: false, body: { error: "dense_session_mismatch", detail: "unknown session" }, contentType: "application/json" };
      beats.push(body);
      return { status: 200, ok: true, body: { ok: true }, contentType: "application/json" };
    },
  };
  return { olp: olp as unknown as OlpClient, beats, timeouts };
}

test("beats for the held session until the window ends, then stops by itself; every beat request carries the 1.5 s bound", async () => {
  let now = 0;
  const { olp, beats, timeouts } = fakeOlp(() => true);
  const keeper = new HeartbeatKeeper(olp, 3000, { intervalMs: 60_000, now: () => now });
  keeper.extend();
  await settle();
  assert.deepEqual(beats.at(-1), { session_id: "s" });
  now = 2999;
  await keeper.beat();
  assert.equal(keeper.state().active, true);
  now = 3000;
  await keeper.beat();
  assert.equal(keeper.state().active, false);
  assert.match(String(keeper.state().ended_reason), /window ended/);
  const sent = beats.length;
  await keeper.beat();
  assert.equal(beats.length, sent, "no beat after the window");
  assert.deepEqual(new Set(timeouts.map(([, ms]) => ms)), new Set([1500]));
});

test("a refused heartbeat re-reads status, and stops when OLP no longer holds a session", async () => {
  let held = true;
  const { olp, beats } = fakeOlp(() => held);
  const keeper = new HeartbeatKeeper(olp, 60_000, { intervalMs: 60_000 });
  keeper.extend();
  await settle();
  held = false;
  await keeper.beat();
  assert.equal(keeper.state().session_id, null, "dense_session_mismatch drops the cached session");
  await keeper.beat();
  assert.equal(keeper.state().active, false);
  assert.match(String(keeper.state().ended_reason), /no held session/);
  assert.ok(beats.length >= 1);
});

test("an action in flight: no stop for an unheld session or an ended window; then success restarts the window, failure and STOP end the beats", async () => {
  let now = 0;
  let held = false;
  const { olp, beats } = fakeOlp(() => held);
  const keeper = new HeartbeatKeeper(olp, 3000, { intervalMs: 60_000, graceMs: 0, now: () => now });
  const pending = () => {
    let finish!: (ok: boolean) => void;
    const promise = keeper.around(() => new Promise<boolean>((resolve) => (finish = resolve)), (ok) => ok);
    return { promise, finish: (ok: boolean) => finish(ok) };
  };

  // Run program before it arms: OLP holds no session yet.
  const first = pending();
  await settle();
  assert.equal(keeper.state().active, true, "an unheld session does not stop the beats while the action is in flight");
  now = 10_000;
  await keeper.beat();
  assert.equal(keeper.state().active, true, "the window cannot end mid-action");
  assert.equal(keeper.state().in_flight, 1);
  // It armed: the next beat finds the session and beats it.
  held = true;
  await keeper.beat();
  assert.deepEqual(beats.at(-1), { session_id: "s" });
  first.finish(true);
  await first.promise;
  assert.equal(keeper.state().remaining_ms, 3000, "success restarts the window from the answer");

  // A refused action on a keeper that was not beating leaves it stopped.
  keeper.stop("test reset");
  held = false;
  const refused = pending();
  await settle();
  refused.finish(false);
  await refused.promise;
  assert.equal(keeper.state().active, false);
  assert.match(String(keeper.state().ended_reason), /did not succeed/);

  // STOP during an action wins over its success.
  const stopped = pending();
  await settle();
  keeper.stop("STOP pressed");
  stopped.finish(true);
  await stopped.promise;
  assert.equal(keeper.state().active, false);
  assert.equal(keeper.state().ended_reason, "STOP pressed");
});

test("overlapping actions: refusals in either order keep nothing alive, a success keeps its own window, an earlier window is kept as it was", async () => {
  let now = 0;
  // OLP holds a session throughout: only the keeper's own bookkeeping can end the beats.
  const { olp, beats } = fakeOlp(() => true);
  const keeper = new HeartbeatKeeper(olp, 3000, { intervalMs: 60_000, graceMs: 0, now: () => now });
  const pending = () => {
    let finish!: (ok: boolean) => void;
    const promise = keeper.around(() => new Promise<boolean>((resolve) => (finish = resolve)), (ok) => ok);
    return async (ok: boolean) => {
      finish(ok);
      await promise;
    };
  };
  const twoInFlight = async () => {
    now += 1000;
    const a = pending();
    await settle();
    now += 100;
    const b = pending();
    await settle();
    assert.equal(keeper.state().in_flight, 2);
    now += 100;
    return { a, b };
  };
  const assertStopped = async (label: string) => {
    assert.equal(keeper.state().active, false, `${label}: no beats once every action in flight refused`);
    assert.match(String(keeper.state().ended_reason), /did not succeed/, label);
    const sent = beats.length;
    await keeper.beat();
    assert.equal(beats.length, sent, `${label}: no beat afterwards`);
  };

  {
    const { a, b } = await twoInFlight();
    await a(false);
    assert.equal(keeper.state().active, true, "the other action is still in flight");
    await b(false);
    await assertStopped("both refused, first one first");
  }
  {
    const { a, b } = await twoInFlight();
    await b(false);
    await a(false);
    await assertStopped("both refused, second one first");
  }
  {
    const { a, b } = await twoInFlight();
    await a(false);
    await b(true);
    assert.equal(keeper.state().remaining_ms, 3000, "refused then succeeded: the window runs from the success's answer");
    keeper.stop("test reset");
  }
  {
    // The success answers in the same millisecond the refused action was sent.
    now += 1000;
    const a = pending();
    await settle();
    now += 50;
    const b = pending();
    await settle();
    await a(true);
    await b(false);
    assert.equal(keeper.state().active, true);
    assert.equal(keeper.state().remaining_ms, 3000, "succeeded then refused: the success's window is not cut back");
    keeper.stop("test reset");
  }
  {
    // An earlier success's window: two refusals neither extend nor end it.
    now += 1000;
    keeper.extend();
    await settle();
    const { a, b } = await twoInFlight();
    await a(false);
    await b(false);
    assert.equal(keeper.state().active, true);
    assert.equal(keeper.state().remaining_ms, 3000 - 1200, "the earlier window ends when it would have");
    now += 1800;
    await keeper.beat();
    assert.equal(keeper.state().active, false);
    assert.match(String(keeper.state().ended_reason), /window ended/);
  }
});

test("a hung heartbeat and status never hold up the next beat: one beat per tick, each request aborted at its bound", async () => {
  let statusCalls = 0;
  let open = 0;
  let maxOpen = 0;
  const arrivals: number[] = [];
  const server = http.createServer((request, response) => {
    if ((request.url ?? "").endsWith("/dense-execution/status") && ++statusCalls === 1) {
      response.writeHead(200, { "Content-Type": "application/json" });
      response.end(JSON.stringify({ available: true, armed: true, session: { id: "s", kind: "authority", state: "idle", lease: { held: true, expires_in_ms: 400 } } }));
      return;
    }
    // Every later status read and every heartbeat hangs: OLP alive on TCP, answering nothing.
    if ((request.url ?? "").endsWith("/dense-execution/heartbeat")) {
      arrivals.push(Date.now());
      open++;
      maxOpen = Math.max(maxOpen, open);
      response.on("close", () => open--);
    }
  });
  server.listen(0, "127.0.0.1");
  await once(server, "listening");
  const { port } = server.address() as { port: number };
  // The production client's 15 s default: the beat must not inherit it.
  const olp = new OlpClient(`http://127.0.0.1:${port}`, "test", 15_000);
  const keeper = new HeartbeatKeeper(olp, 60_000, { intervalMs: 50, requestTimeoutMs: 200, statusRefreshMs: 100 });
  try {
    keeper.extend();
    await sleep(800);
  } finally {
    keeper.stop("test ended");
    server.closeAllConnections();
    server.close();
  }
  assert.ok(arrivals.length >= 8, `a beat every 50 ms for 0.8 s, saw ${arrivals.length}`);
  const gaps = arrivals.slice(1).map((t, i) => t - arrivals[i]!);
  assert.ok(Math.max(...gaps) <= 200, `no gap between beats beyond the 200 ms request bound, saw ${Math.max(...gaps)} ms`);
  assert.ok(maxOpen <= 6, `hung beats are aborted at 200 ms (about 4 open at 50 ms), saw ${maxOpen} open`);
  assert.ok(keeper.state().last_error, "the hung requests are reported");
});
