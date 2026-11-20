// The bounded jog tools through the SDK client, against a fake OLP at the
// HTTP boundary: which modes offer them, that a real-robot target under
// `simulation` gets no POST, the exact bodies and fence headers the browser
// sends, this server's caps refusing before anything is sent, and the hold
// always ending (End, then Stop when it failed). Angles in degrees and
// distances in mm, written out here, not derived from the code under test.
import assert from "node:assert/strict";
import { after, before, beforeEach, describe, test } from "node:test";

import { connect, simTarget, startFakeOlp, textOf, type FakeOlp, type Recorded } from "./helpers.js";

const JOG_TOOLS = ["rosie_olp_move_joints", "rosie_olp_cartesian_step", "rosie_olp_jog_hold"];
const rad = (deg: number) => (deg * Math.PI) / 180;
// Live joint angles of the fake cell: J1 10°, J2 -20°, J3 0°, J4..J6 0°.
const LIVE_DEG = [10, -20, 0, 0, 0, 0];
const REVISION = 41;

function machineStatus(simulation: boolean, extra: Record<string, unknown> = {}) {
  return {
    available: true,
    backend: "rt_core",
    target: simTarget(simulation),
    armed: true,
    session: { id: "sess-1", kind: "authority", state: "idle", lease: { held: true, expires_in_ms: 400 }, heartbeat_age_ms: 100, heartbeat_deadline_ms: 5000 },
    cursor: null,
    axes: [],
    loop: null,
    last_jog_end: { seq: 3, code: "released", text: "Jog released.", repull: "yes" },
    rt_core: { status: { axes: LIVE_DEG.map((deg) => ({ logical_valid: true, logical_position: rad(deg) })) } },
    ...extra,
  };
}

// max_velocity 1 rad/s (57.30°/s) on every joint: a hold at fraction f runs
// at f x 0.75 x 57.30°/s in OLP.
const capabilities = {
  backend: "rt_core",
  cell: "local-dev",
  available: true,
  rt_core: {
    description: {
      axes: LIVE_DEG.map((_, index) => ({ index, id: `J${index + 1}`, position_unit: "rad", min_position: rad(-170), max_position: rad(170), max_velocity: 1, jog_acceleration: 0.5 })),
    },
  },
};

const jogState = { active: false, revision: REVISION, generation: 0, sequence: 0, axis: 0, reason: "", cadence_ms: 50, pipelined_intents: true, move: { axis: 0, state: "done", reason: "", revision: REVISION } };

const posts = (olp: FakeOlp, prefix = "/dense-execution/") =>
  olp.requests.filter((r) => r.method === "POST" && r.path.startsWith(prefix) && r.path !== "/dense-execution/heartbeat");

function fenced(request: Recorded | undefined): void {
  assert.ok(request, "request reached OLP");
  assert.equal(request.headers["x-rt-target-generation"], "7");
  assert.equal(request.headers["x-rt-target-cell"], "local-sim");
  assert.match(String(request.headers["x-rosie-client-id"]), /^mcp-[0-9a-f]{16}$/);
}

describe("bounded jog tools", () => {
  let olp: FakeOlp;
  before(async () => {
    olp = await startFakeOlp(machineStatus(true));
  });
  after(async () => {
    await olp.close();
  });
  beforeEach(() => {
    olp.status = machineStatus(true);
    olp.requests.length = 0;
    olp.routes = {
      "/dense-execution/capabilities": () => [200, capabilities],
      "/dense-execution/jog/state": () => [200, jogState],
      "/dense-execution/cartesian/state": () => [200, jogState],
      "/dense-execution/move": (r) => [200, { axis: (r.body as { axis: number }).axis, state: "done", reason: "", revision: REVISION + 1 }],
      "/dense-execution/cartesian/move": (r) => [200, { axis: (r.body as { axis: number }).axis, state: "done", reason: "", revision: REVISION + 1 }],
      "/dense-execution/jog/begin": () => [200, { ...jogState, active: true }],
      "/dense-execution/jog/update": () => [200, { ...jogState, active: true }],
      "/dense-execution/jog/end": () => [200, { ...jogState, revision: REVISION + 1 }],
      "/dense-execution/cartesian/start": () => [200, { ...jogState, active: true }],
      "/dense-execution/cartesian/intent": () => [200, { ...jogState, active: true }],
      "/dense-execution/cartesian/halt": () => [200, { ...jogState, revision: REVISION + 1 }],
    };
  });

  test("off offers none of them; simulation and cell do", async () => {
    for (const [mode, offered] of [["off", false], ["simulation", true], ["cell", true]] as const) {
      const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: mode });
      try {
        const names = (await session.client.listTools()).tools.map((tool) => tool.name);
        for (const name of JOG_TOOLS) assert.equal(names.includes(name), offered, `${mode}: ${name}`);
        assert.deepEqual(names.filter((name) => /home|reset|recover/.test(name)), [], mode);
      } finally {
        await session.close();
      }
    }
  });

  test("simulation: a real-robot target gets no POST from any of them", async () => {
    olp.status = machineStatus(false);
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "simulation" });
    try {
      const calls = [
        { name: "rosie_olp_move_joints", arguments: { mode: "relative", joints: [{ joint: "J1", deg: 1 }] } },
        { name: "rosie_olp_cartesian_step", arguments: { axis: "x", amount: 5 } },
        { name: "rosie_olp_jog_hold", arguments: { kind: "joint", joint: "J1", direction: "positive", duration_s: 0.2 } },
      ];
      for (const call of calls) {
        const result = await session.client.callTool(call);
        assert.equal(result.isError, true, call.name);
        assert.match(textOf(result), /not a simulation/);
      }
      assert.deepEqual(olp.requests.filter((r) => r.method === "POST").map((r) => r.path), []);
    } finally {
      await session.close();
    }
  });

  test("relative move of one joint: the GO button's body, fenced, revision from jog/state, target_id from capabilities", async () => {
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "simulation" });
    try {
      const result = await session.client.callTool({ name: "rosie_olp_move_joints", arguments: { mode: "relative", joints: [{ joint: "j2", deg: 5 }] } });
      assert.notEqual(result.isError, true, textOf(result));
      const sent = posts(olp);
      assert.deepEqual(sent.map((r) => r.path), ["/dense-execution/move"]);
      assert.deepEqual(sent[0]!.body, { target_id: "local-dev", session_id: "sess-1", revision: REVISION, axis: 1, degrees: 5, fraction: 0.1 });
      fenced(sent[0]);
      assert.match(textOf(result), /J2: -20\.00° → -15\.00°/);
    } finally {
      await session.close();
    }
  });

  test("absolute move: one listed move with position_rad, travel bounded from the LIVE position, joints already there left out", async () => {
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "simulation" });
    try {
      // J1 is at 10°: -25° is 35° of travel (over 30°) although |-25°| < 30°.
      const far = await session.client.callTool({ name: "rosie_olp_move_joints", arguments: { mode: "absolute", joints: [{ joint: "J1", deg: -25 }] } });
      assert.equal(far.isError, true);
      assert.match(textOf(far), /J1 would travel -35\.00°.*30° per joint per call/);
      assert.deepEqual(posts(olp), []);

      // 39° is 29° of travel from 10° (allowed although 39° > 30°); J2 -20° -> -15°; J3 already at 0°.
      const result = await session.client.callTool({
        name: "rosie_olp_move_joints",
        arguments: { mode: "absolute", joints: [{ joint: "J1", deg: 39 }, { joint: "J2", deg: -15 }, { joint: "J3", deg: 0 }], fraction: 0.25 },
      });
      assert.notEqual(result.isError, true, textOf(result));
      const sent = posts(olp);
      assert.deepEqual(sent.map((r) => r.path), ["/dense-execution/move"]);
      const body = sent[0]!.body as { axes: Array<{ axis: number; degrees: number; position_rad: number }> } & Record<string, unknown>;
      assert.deepEqual({ ...body, axes: undefined }, { target_id: "local-dev", session_id: "sess-1", revision: REVISION, axis: 0, degrees: 0, fraction: 0.25, axes: undefined });
      assert.deepEqual(body.axes.map((a) => [a.axis, a.degrees]), [[0, 0], [1, 0]]);
      assert.ok(Math.abs(body.axes[0]!.position_rad - (39 * Math.PI) / 180) < 1e-12);
      assert.ok(Math.abs(body.axes[1]!.position_rad - (-15 * Math.PI) / 180) < 1e-12);
      fenced(sent[0]);
      assert.match(textOf(result), /J1: 10\.00° → 39\.00°/);
      assert.match(textOf(result), /J2: -20\.00° → -15\.00°/);
      assert.doesNotMatch(textOf(result), /J3/);
    } finally {
      await session.close();
    }
  });

  test("this server's caps refuse before any POST: 31°, 101 mm, 16°, a hold commanding 32.2°", async () => {
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "cell" });
    try {
      const refusedCalls = [
        { name: "rosie_olp_move_joints", arguments: { mode: "relative", joints: [{ joint: "J4", deg: 31 }] } },
        { name: "rosie_olp_move_joints", arguments: { mode: "relative", joints: [{ joint: "J1", deg: 5 }, { joint: "J1", deg: 5 }] } },
        { name: "rosie_olp_cartesian_step", arguments: { axis: "z", amount: -101 } },
        { name: "rosie_olp_cartesian_step", arguments: { axis: "rz", amount: 16 } },
        // 57.30°/s x 0.25 x 0.75 x 3 s = 32.2° of J1 > 30°.
        { name: "rosie_olp_jog_hold", arguments: { kind: "joint", joint: "J1", direction: "negative", duration_s: 3, fraction: 0.25 } },
        // 1000 mm/s x 0.25 x 0.5 s = 125 mm > 100 mm.
        { name: "rosie_olp_jog_hold", arguments: { kind: "cartesian", axis: "y", direction: "positive", duration_s: 0.5, fraction: 0.25, speed_mm_s: 1000 } },
        { name: "rosie_olp_jog_hold", arguments: { kind: "joint", joint: "J1", direction: "positive", duration_s: 3.5 } },
        { name: "rosie_olp_move_joints", arguments: { mode: "relative", joints: [{ joint: "J1", deg: 1 }], fraction: 0.6 } },
      ];
      for (const call of refusedCalls) {
        // A schema bound (fraction, duration) may come back as a protocol error rather than a tool result.
        const result = await session.client.callTool(call).catch((error: unknown) => ({ isError: true, thrown: error }));
        assert.equal(result.isError, true, `${call.name} ${JSON.stringify(call.arguments)}`);
      }
      assert.deepEqual(posts(olp), []);
      assert.equal(session.ctx.keeper.state().active, false, "nothing was sent, nothing is kept alive");
    } finally {
      await session.close();
    }
  });

  test("Cartesian step: the browser's step body in the tool frame, fenced", async () => {
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "simulation" });
    try {
      const result = await session.client.callTool({ name: "rosie_olp_cartesian_step", arguments: { frame: "tool", axis: "z", amount: -25 } });
      assert.notEqual(result.isError, true, textOf(result));
      const sent = posts(olp);
      assert.deepEqual(sent.map((r) => r.path), ["/dense-execution/cartesian/move"]);
      assert.deepEqual(sent[0]!.body, { target_id: "local-dev", session_id: "sess-1", revision: REVISION, frame: "tool", axis: 2, distance_mm: -25, angle_degrees: 0, fraction: 0.1 });
      fenced(sent[0]);
      assert.ok(olp.requests.some((r) => r.method === "GET" && r.path === "/dense-execution/cartesian/state"));
    } finally {
      await session.close();
    }
  });

  test("joint hold: begin, the same intent every 50 ms, then end with the release cause; OLP's end reason reported", async () => {
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "simulation" });
    try {
      const result = await session.client.callTool({ name: "rosie_olp_jog_hold", arguments: { kind: "joint", joint: "J3", direction: "positive", duration_s: 0.4 } });
      assert.notEqual(result.isError, true, textOf(result));
      const sent = posts(olp);
      const intent = { target_id: "local-dev", session_id: "sess-1", revision: REVISION, axis: 2, direction: 1, fraction: 0.1 };
      assert.equal(sent[0]!.path, "/dense-execution/jog/begin");
      assert.deepEqual(sent[0]!.body, intent);
      const updates = sent.filter((r) => r.path === "/dense-execution/jog/update");
      assert.ok(updates.length >= 4, `0.4 s at 50 ms is ~8 intents, saw ${updates.length}`);
      for (const update of updates) assert.deepEqual(update.body, intent);
      const end = sent.find((r) => r.path === "/dense-execution/jog/end");
      assert.deepEqual(end?.body, { ...intent, end_cause: "button_released" });
      fenced(end);
      assert.ok(sent.indexOf(end!) > sent.indexOf(updates[0]!));
      assert.equal(sent.some((r) => r.path === "/dense-execution/stop"), false);
      assert.match(textOf(result), /OLP's jog end: released: Jog released\./);
    } finally {
      await session.close();
    }
  });

  test("Cartesian hold: start/intent/halt with the browser's twist (100 mm/s at 100% is 0.1 m/s; the fraction travels beside it)", async () => {
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "simulation" });
    try {
      const result = await session.client.callTool({ name: "rosie_olp_jog_hold", arguments: { kind: "cartesian", axis: "x", frame: "base", direction: "negative", duration_s: 0.2 } });
      assert.notEqual(result.isError, true, textOf(result));
      const sent = posts(olp);
      const intent = { target_id: "local-dev", session_id: "sess-1", revision: REVISION, frame: "base", twist: [-0.1, 0, 0, 0, 0, 0], fraction: 0.1 };
      assert.deepEqual(sent[0]!.body, intent);
      assert.equal(sent[0]!.path, "/dense-execution/cartesian/start");
      assert.ok(sent.some((r) => r.path === "/dense-execution/cartesian/intent"));
      const halt = sent.find((r) => r.path === "/dense-execution/cartesian/halt");
      assert.deepEqual(halt?.body, { ...intent, end_cause: "button_released" });
      fenced(halt);
    } finally {
      await session.close();
    }
  });

  test("an intent refused mid-hold: End carries OLP's code, then Stop", async () => {
    olp.routes["/dense-execution/jog/update"] = () => [409, { error: "no_grant", detail: "the grant was lost" }];
    olp.routes["/dense-execution/jog/end"] = () => [409, { error: "jog_session_stale", detail: "revision retired" }];
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "simulation" });
    try {
      const result = await session.client.callTool({ name: "rosie_olp_jog_hold", arguments: { kind: "joint", joint: "J1", direction: "negative", duration_s: 2 } });
      assert.equal(result.isError, true);
      assert.match(textOf(result), /no_grant: the grant was lost/);
      const paths = posts(olp).map((r) => r.path);
      const end = paths.indexOf("/dense-execution/jog/end");
      assert.ok(end > 0, "End was sent");
      assert.deepEqual((posts(olp)[end]!.body as { end_cause: string }).end_cause, "intent_refused:no_grant");
      assert.ok(paths.indexOf("/dense-execution/stop") > end, "Stop after End");
      assert.equal(session.ctx.keeper.state().active, false);
    } finally {
      await session.close();
    }
  });

  test("an End that fails gets Stop", async () => {
    olp.routes["/dense-execution/jog/end"] = () => [500, { error: "rt_core_failure", detail: "end uncertain" }];
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "simulation" });
    try {
      const result = await session.client.callTool({ name: "rosie_olp_jog_hold", arguments: { kind: "joint", joint: "J1", direction: "positive", duration_s: 0.1 } });
      assert.equal(result.isError, true);
      const paths = posts(olp).map((r) => r.path);
      assert.ok(paths.indexOf("/dense-execution/stop") > paths.indexOf("/dense-execution/jog/end"));
      assert.match(textOf(result), /STOP: sent because End did not end the hold; confirmed/);
    } finally {
      await session.close();
    }
  });

  test("OLP refuses the press while a loop runs: reported in its words, End sent, no Stop (Stop would end the loop)", async () => {
    olp.status = machineStatus(true, { loop: { mode: "count", state: "running", cycles: 2 } });
    olp.routes["/dense-execution/jog/begin"] = () => [409, { error: "jog_mode_conflict", detail: "jog_mode_conflict" }];
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "simulation" });
    try {
      const result = await session.client.callTool({ name: "rosie_olp_jog_hold", arguments: { kind: "joint", joint: "J1", direction: "positive", duration_s: 1 } });
      assert.equal(result.isError, true);
      assert.match(textOf(result), /not started: OLP refused the press \(HTTP 409\): jog_mode_conflict/);
      assert.match(textOf(result), /A program loop is running/);
      const paths = posts(olp).map((r) => r.path);
      assert.ok(paths.includes("/dense-execution/jog/end"));
      assert.equal(paths.includes("/dense-execution/stop"), false);
    } finally {
      await session.close();
    }
  });

  test("a refused press whose End is refused too (jog_mode_conflict while a program plays): End sent, still no Stop", async () => {
    olp.status = machineStatus(true, { session: { ...machineStatus(true).session, state: "playing" } });
    olp.routes["/dense-execution/jog/begin"] = () => [409, { error: "jog_mode_conflict", detail: "jog_mode_conflict" }];
    olp.routes["/dense-execution/jog/end"] = () => [409, { error: "jog_mode_conflict", detail: "jog_mode_conflict" }];
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "simulation" });
    try {
      const result = await session.client.callTool({ name: "rosie_olp_jog_hold", arguments: { kind: "joint", joint: "J2", direction: "negative", duration_s: 0.5 } });
      assert.equal(result.isError, true);
      assert.match(textOf(result), /not started: OLP refused the press/);
      assert.match(textOf(result), /A program is playing/);
      const paths = posts(olp).map((r) => r.path);
      assert.deepEqual(paths.filter((p) => p !== "/dense-execution/jog/update"), ["/dense-execution/jog/begin", "/dense-execution/jog/end"]);
      assert.doesNotMatch(textOf(result), /STOP/);
      assert.equal(session.ctx.keeper.state().active, false, "a refused press keeps nothing alive");
    } finally {
      await session.close();
    }
  });

  test("an accepted hold whose End is refused with anything but jog_session_stale gets Stop", async () => {
    olp.routes["/dense-execution/jog/end"] = () => [409, { error: "jog_mode_conflict", detail: "jog_mode_conflict" }];
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "simulation" });
    try {
      const result = await session.client.callTool({ name: "rosie_olp_jog_hold", arguments: { kind: "joint", joint: "J2", direction: "negative", duration_s: 0.1 } });
      assert.equal(result.isError, true);
      const paths = posts(olp).map((r) => r.path);
      assert.ok(paths.indexOf("/dense-execution/stop") > paths.indexOf("/dense-execution/jog/end"), "Stop after End");
      assert.match(textOf(result), /STOP: sent because End did not end the hold/);
    } finally {
      await session.close();
    }
  });

  test("a refused move is OLP's answer, not retried", async () => {
    olp.routes["/dense-execution/move"] = () => [409, { error: "joint_move_limits", detail: "J1 endpoint outside its range" }];
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "simulation" });
    try {
      const result = await session.client.callTool({ name: "rosie_olp_move_joints", arguments: { mode: "relative", joints: [{ joint: "J1", deg: -3 }] } });
      assert.equal(result.isError, true);
      assert.match(textOf(result), /OLP refused joint move \(HTTP 409\): joint_move_limits: J1 endpoint outside its range/);
      assert.equal(posts(olp).filter((r) => r.path === "/dense-execution/move").length, 1);
    } finally {
      await session.close();
    }
  });
});
