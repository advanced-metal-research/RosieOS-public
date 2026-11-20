// Through the SDK client, as a host calls it: which tools exist in each
// robot-action mode, what reaches OLP (and what must not), and the offline
// tools against this checkout's committed files.
import assert from "node:assert/strict";
import { existsSync, readFileSync } from "node:fs";
import path from "node:path";
import { after, before, describe, test } from "node:test";

import { connect, REPO_ROOT, simTarget, startFakeOlp, startFakeRtControl, textOf, type FakeOlp } from "./helpers.js";

// Bounded jogs are offered in simulation/cell mode (test/jog.test.ts); Home, reset and recover never are.
const NEVER_OFFERED = /home|reset|recover|test_program|upload|deploy_release$|rollback|reboot|install|restart/;
const fixture = path.join(REPO_ROOT, "motion-server", "joint-trajectory", "v1", "testdata", "dense_joint_trajectory_reference.rdt");
const expectedDigest = (JSON.parse(readFileSync(fixture.replace(".rdt", ".expected.json"), "utf8")) as { trajectory_digest: string }).trajectory_digest;

function armedStatus(simulation: boolean) {
  return {
    available: true,
    backend: "rt_core",
    target: simTarget(simulation),
    armed: true,
    session: { id: "sess-1", kind: "authority", state: "idle", lease: { held: true, expires_in_ms: 400 }, heartbeat_age_ms: 100, heartbeat_deadline_ms: 5000 },
    cursor: null,
    axes: [],
  };
}

describe("robot-action modes", () => {
  let olp: FakeOlp;
  before(async () => {
    olp = await startFakeOlp(armedStatus(true));
  });
  after(async () => {
    await olp.close();
  });

  test("off: only the tools that end motion; Home, reset and installs are never offered", async () => {
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "off" });
    try {
      const names = (await session.client.listTools()).tools.map((tool) => tool.name);
      for (const name of ["rosie_olp_stop", "rosie_olp_disarm", "rosie_olp_pause", "rosie_olp_finish_loop"]) assert.ok(names.includes(name), name);
      for (const name of ["rosie_olp_arm", "rosie_olp_play", "rosie_olp_load", "rosie_olp_run_program", "rosie_olp_select_cell"]) assert.ok(!names.includes(name), name);
      assert.deepEqual(names.filter((name) => NEVER_OFFERED.test(name)), []);
    } finally {
      await session.close();
    }
  });

  test("simulation: Arm on a real-robot target is refused here and never reaches OLP", async () => {
    olp.status = armedStatus(false);
    olp.requests.length = 0;
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "simulation" });
    try {
      const names = (await session.client.listTools()).tools.map((tool) => tool.name);
      assert.ok(names.includes("rosie_olp_arm"));
      assert.deepEqual(names.filter((name) => NEVER_OFFERED.test(name)), []);
      const result = await session.client.callTool({ name: "rosie_olp_arm", arguments: {} });
      assert.equal(result.isError, true);
      assert.match(textOf(result), /not a simulation/);
      assert.deepEqual(olp.requests.filter((r) => r.method === "POST").map((r) => r.path), []);
    } finally {
      await session.close();
    }
  });

  test("simulation: Arm on a simulated target carries the status read's selection fence and our client id, then heartbeats", async () => {
    olp.status = armedStatus(true);
    olp.requests.length = 0;
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "simulation" });
    try {
      const result = await session.client.callTool({ name: "rosie_olp_arm", arguments: {} });
      assert.notEqual(result.isError, true, textOf(result));
      const arm = olp.requests.find((r) => r.path === "/dense-execution/arm");
      assert.ok(arm, "arm reached OLP");
      assert.deepEqual(arm.body, { armed: true });
      assert.equal(arm.headers["x-rt-target-generation"], "7");
      assert.equal(arm.headers["x-rt-target-cell"], "local-sim");
      assert.match(String(arm.headers["x-rosie-client-id"]), /^mcp-[0-9a-f]{16}$/);
      assert.match(String(arm.headers["x-rosie-client"]), /^rosieos-mcp-server/);
      for (let i = 0; i < 50 && !olp.requests.some((r) => r.path === "/dense-execution/heartbeat"); i++) await new Promise((r) => setTimeout(r, 20));
      const beat = olp.requests.find((r) => r.path === "/dense-execution/heartbeat");
      assert.deepEqual(beat?.body, { session_id: "sess-1" });
      assert.equal(session.ctx.keeper.state().active, true);

      const stop = await session.client.callTool({ name: "rosie_olp_stop", arguments: {} });
      assert.notEqual(stop.isError, true);
      assert.match(textOf(stop), /STOP confirmed/);
      assert.equal(session.ctx.keeper.state().active, false, "STOP ends this server's heartbeats");
    } finally {
      await session.close();
    }
  });

  test("Stop needs no target fence and works with robot actions off", async () => {
    olp.requests.length = 0;
    const session = await connect({ ROSIE_OLP_URL: olp.url, ROSIE_MCP_ROBOT_ACTIONS: "off" });
    try {
      await session.client.callTool({ name: "rosie_olp_stop", arguments: {} });
      const stop = olp.requests.find((r) => r.path === "/dense-execution/stop");
      assert.ok(stop);
      assert.equal(stop.headers["x-rt-target-generation"], undefined);
    } finally {
      await session.close();
    }
  });
});

describe("rt-core observation", () => {
  test("Status over the public socket: envelope unwrapped, axes named from Describe, alarm shown", async () => {
    const rt = await startFakeRtControl({
      "/v1/describe": {
        backend: "simulation",
        protocol_major: 1,
        protocol_minor: 14,
        configuration_sha256: "a".repeat(64),
        machine_sha256: "b".repeat(64),
        deployment_sha256: "c".repeat(64),
        cycle_ns: 1_000_000,
        axes: [{ index: 0, id: "J1", position_unit: "rad", min_position: -3, max_position: 3, max_velocity: 2, require_home: true, native_home: true, completion_tolerance: 0.001 }],
      },
      "/v1/status": {
        daemon_incarnation: "d",
        adapter_incarnation: "a",
        grant: null,
        core: { backend: "simulation", configuration_sha256: "a".repeat(64), armed: 0, lease_valid: 0, home_valid_mask: 1, safety_fault_mask: 0, execution_fault_reasons: 0, axis_enable_mask: 0, recovery: null },
        axes: [{ readiness: "faulted", logical_position: Math.PI / 2, logical_valid: true, logical_target: Math.PI / 2, home_valid: true, drive_alarm: "Er41.1", drive_alarm_text: "Motor over-temperature", statusword: 0, following_error_counts: 0, following_error_valid: true, torque_raw: 0, brake_state: 0 }],
      },
    });
    const session = await connect({ ROSIE_OLP_URL: "http://127.0.0.1:9", ROSIE_RT_CONTROL_SOCKET: rt.socketPath });
    try {
      const result = await session.client.callTool({ name: "rosie_rtcore_get_status", arguments: {} });
      assert.notEqual(result.isError, true, textOf(result));
      const text = textOf(result);
      assert.match(text, /\| J1 \| faulted \| 90\.000° /);
      assert.match(text, /Er41\.1 Motor over-temperature/);
      assert.equal((result.structuredContent as { core: { backend: string } }).core.backend, "simulation");
    } finally {
      await session.close();
      await rt.close();
    }
  });

  test("without a configured socket the tool says how to configure it", async () => {
    const session = await connect({ ROSIE_OLP_URL: "http://127.0.0.1:9" });
    try {
      const result = await session.client.callTool({ name: "rosie_rtcore_describe", arguments: {} });
      assert.equal(result.isError, true);
      assert.match(textOf(result), /ROSIE_RT_CONTROL_SOCKET/);
    } finally {
      await session.close();
    }
  });
});

describe("offline tools and resources (this checkout)", () => {
  test("inspect the reference .rdt: same digest as the arbiter", async () => {
    const session = await connect({});
    try {
      const result = await session.client.callTool({ name: "rosie_inspect_dense_trajectory", arguments: { path: fixture, response_format: "json" } });
      assert.notEqual(result.isError, true, textOf(result));
      assert.equal((result.structuredContent as { identity: { trajectory_digest: string } }).identity.trajectory_digest, expectedDigest);
    } finally {
      await session.close();
    }
  });

  test("Er41.1 is found exactly and classed by the drive profile's resettable list", async () => {
    const profile = JSON.parse(readFileSync(path.join(REPO_ROOT, "rt-core", "config", "drives", "a6ec.json"), "utf8")) as { fault_reset: { resettable_alarms: Array<{ code: string }> } };
    const listed = profile.fault_reset.resettable_alarms.some((alarm) => alarm.code === "Er41.1");
    const session = await connect({});
    try {
      const result = await session.client.callTool({ name: "rosie_lookup_drive_alarm", arguments: { query: "er41.1", response_format: "json" } });
      const first = (result.structuredContent as { matches: Array<{ code: string; exact: boolean; rt_core_profile_resettable: boolean }> }).matches[0]!;
      assert.deepEqual([first.code, first.exact, first.rt_core_profile_resettable], ["Er41.1", true, listed]);
    } finally {
      await session.close();
    }
  });

  // A real cell: present in the private repository, left out of the public mirror.
  const armCell = path.join(REPO_ROOT, "rt-core", "config", "cells", "example-arm-1420.json");
  test("cells list carries rosie_1420_v1's committed pin; manifest sections are readable", { skip: !existsSync(armCell) && "private site configuration not present" }, async () => {
    const manifest = JSON.parse(readFileSync(armCell, "utf8")) as { nodes: Array<{ rt_core?: { configuration_sha256: string } }> };
    const pin = manifest.nodes.find((node) => node.rt_core)!.rt_core!.configuration_sha256;
    const session = await connect({});
    try {
      const result = await session.client.callTool({ name: "rosie_list_cells", arguments: { response_format: "json" } });
      const cells = (result.structuredContent as { cells: Array<{ file: string; configuration_sha256: string }> }).cells;
      assert.equal(cells.find((cell) => cell.file === "example-arm-1420")?.configuration_sha256, pin);
      const missing = await session.client.callTool({ name: "rosie_get_cell_manifest", arguments: { cell: "example-arm-1420", section: "no_such_section" } });
      assert.equal(missing.isError, true);
    } finally {
      await session.close();
    }
  });

  test("docs: search finds the HM35 rule; every listed document resource reads", async () => {
    const session = await connect({});
    try {
      const search = await session.client.callTool({ name: "rosie_search_docs", arguments: { query: "HM35", docs: ["repo-rules"], response_format: "json" } });
      assert.ok((search.structuredContent as { total: number }).total > 0);
      const { resources } = await session.client.listResources();
      const docs = resources.filter((resource) => resource.uri.startsWith("rosieos://docs/"));
      assert.ok(docs.length >= 20);
      for (const doc of docs) {
        const read = await session.client.readResource({ uri: doc.uri });
        assert.ok(String((read.contents[0] as { text?: string }).text ?? "").length > 0, doc.uri);
      }
      const prompts = (await session.client.listPrompts()).prompts.map((prompt) => prompt.name);
      assert.deepEqual(prompts.sort(), ["rosie_author_weld_program", "rosie_diagnose_cell", "rosie_review_release", "rosie_run_program_in_simulation"]);
    } finally {
      await session.close();
    }
  });
});
