// Run program is one request that can arm, take the grant and move to the
// plan start before it answers (OLP waits up to 300 s). OLP's 5 s deadman
// applies the whole time, so this server must heartbeat DURING the request,
// on a single call and on the confirmed second call of the two-step
// conversation, when the keeper stopped itself after the first. Against a
// fake OLP at the HTTP boundary; the keeper beats every 50 ms and the slow
// answer takes 0.6 s (twelve beats; the real case is 1 s beats over minutes).
import assert from "node:assert/strict";
import { after, before, beforeEach, describe, test } from "node:test";

import { connect, simTarget, startFakeOlp, textOf, type FakeOlp, type Recorded } from "./helpers.js";

const sleep = (ms: number) => new Promise((resolve) => setTimeout(resolve, ms));
const TIMING = { intervalMs: 50, statusRefreshMs: 50, graceMs: 100 };
const ANSWER_MS = 600;
const ARMS_AFTER_MS = 150;

const identity = {
  trajectory_digest: `sha256:${"ab".repeat(32)}`,
  plan_id: "plan-1",
  program_id: "program-1",
  program_digest: "digest-1",
  manifest_revision: 3,
  plan_revision: 2,
};
const joints = [{ axis: 0, id: "J1", from_deg: 10, to_deg: 0, delta_deg: -10, moves: true }];

const disarmed = () => ({ available: true, backend: "rt_core", target: simTarget(true), armed: false, session: null, cursor: null, axes: [] });
const running = () => ({
  ...disarmed(),
  armed: true,
  session: { id: "sess-run", kind: "trajectory", state: "playing", lease: { held: true, expires_in_ms: 400 }, heartbeat_age_ms: 100, heartbeat_deadline_ms: 5000 },
});

describe("run program is beaten around its request", () => {
  let olp: FakeOlp;
  /** Heartbeats that reached OLP while each run-program request was open. */
  let during: Recorded[][];

  /** A run-program answer after ANSWER_MS that arms (status turns held) after ARMS_AFTER_MS, as OLP's Arm-and-run does. */
  const slowRun = (answer: Record<string, unknown>) => async () => {
    const from = olp.requests.length;
    setTimeout(() => (olp.status = running()), ARMS_AFTER_MS);
    await sleep(ANSWER_MS);
    during.push(olp.requests.slice(from).filter((r) => r.path === "/dense-execution/heartbeat"));
    return [200, answer] as [number, unknown];
  };

  before(async () => {
    olp = await startFakeOlp(disarmed());
  });
  after(async () => {
    await olp.close();
  });
  beforeEach(() => {
    olp.status = disarmed();
    olp.requests.length = 0;
    olp.routes = {};
    during = [];
  });

  test("one call that arms and moves before answering: heartbeats land while it is in flight", async () => {
    olp.routes["/dense-execution/run-program"] = slowRun({ state: "playing", session_id: "sess-run", steps: ["arm", "move_to_start", "load", "play"] });
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "simulation" }, { keeper: TIMING });
    try {
      const result = await session.client.callTool({ name: "rosie_olp_run_program", arguments: identity });
      assert.notEqual(result.isError, true, textOf(result));
      const beats = during[0]!;
      // 0.45 s held of the 0.6 s request at 50 ms: about nine beats.
      assert.ok(beats.length >= 3, `heartbeats during the request, saw ${beats.length}`);
      for (const beat of beats) assert.deepEqual(beat.body, { session_id: "sess-run" });
      assert.equal(session.ctx.keeper.state().active, true, "the window runs on after success");
    } finally {
      await session.close();
    }
  });

  test("the confirmed second call: the keeper stopped after 'confirm', and still beats during the confirmed request", async () => {
    olp.routes["/dense-execution/run-program"] = async () => [200, { state: "confirm", action: "Move to program start and run", needs_move: true, needs_arm: true, joints }];
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "simulation" }, { keeper: TIMING });
    try {
      const ask = await session.client.callTool({ name: "rosie_olp_run_program", arguments: identity });
      assert.match(textOf(ask), /needs the operator's confirmation/);
      // Nothing is held after 'confirm': the keeper ends itself once the grace is over.
      for (let i = 0; i < 40 && session.ctx.keeper.state().active; i++) await sleep(25);
      assert.equal(session.ctx.keeper.state().active, false);
      assert.match(String(session.ctx.keeper.state().ended_reason), /no held session/);

      olp.routes["/dense-execution/run-program"] = slowRun({ state: "playing", session_id: "sess-run", steps: ["arm", "move_to_start", "load", "play"] });
      const result = await session.client.callTool({
        name: "rosie_olp_run_program",
        arguments: { ...identity, confirm_arm: true, confirm_move: true, confirmed: joints },
      });
      assert.notEqual(result.isError, true, textOf(result));
      const beats = during[0]!;
      assert.ok(beats.length >= 3, `heartbeats during the confirmed request, saw ${beats.length}`);
      for (const beat of beats) assert.deepEqual(beat.body, { session_id: "sess-run" });
    } finally {
      await session.close();
    }
  });

  test("a refused run keeps nothing alive", async () => {
    olp.routes["/dense-execution/run-program"] = async () => [409, { error: "home_invalid", detail: "J2 Home is not established", run: { refusal: { step: "arm" } } }];
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "simulation" }, { keeper: TIMING });
    try {
      const result = await session.client.callTool({ name: "rosie_olp_run_program", arguments: identity });
      assert.equal(result.isError, true);
      assert.match(textOf(result), /stopped at 'arm': home_invalid/);
      assert.equal(session.ctx.keeper.state().active, false);
    } finally {
      await session.close();
    }
  });
});
