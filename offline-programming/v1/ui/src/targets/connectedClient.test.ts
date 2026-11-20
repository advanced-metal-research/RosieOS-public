import assert from "node:assert/strict";
import test from "node:test";

import {
  CONNECTED_TARGETS_PATH,
  ConnectedTargetsClient,
} from "./connectedClient.ts";

function discovery(profileId = "bench-relay"): Record<string, unknown> {
  return {
    schema: "offline-programming.connected-target-discovery.v1",
    profile_id: profileId,
    manifest_ref: `rt-core/config/cells/${profileId}.json`,
    manifest_schema: "robot_v4_deployment_manifest_v1",
    cell_id: profileId,
    peer_stale_after_s: 120,
    robot_model_bound: false,
    services: [
      {
        role: "motion-server",
        status_stream: true,
        plan_stream: true,
        read_only: true,
        server_mediated: true,
      },
      {
        role: "realtime-core",
        status_stream: true,
        plan_stream: false,
        read_only: true,
        server_mediated: true,
      },
    ],
    target: {
      schema: "offline-programming.target-descriptor.v1",
      profile_id: profileId,
      label: profileId,
      kind: "real_cell",
      manifest_ref: `rt-core/config/cells/${profileId}.json`,
      manifest_schema: "robot_v4_deployment_manifest_v1",
      cell_id: profileId,
      axes: Array.from({ length: 9 }, (_, index) => ({
        index,
        name: `J${index + 1}`,
        urdf_joint: `J${index + 1}`,
        kind: index < 6 ? "arm" : "external",
        gear_ratio: 10,
        sign: 1,
        cartesian_participates: index < 6,
      })),
      capabilities: {
        monitoring: true,
        planning: true,
        simulation: false,
        real_execution: false,
        teleop: false,
        raw_browser_transport: false,
      },
    },
  };
}

function response(value: unknown): Response {
  return new Response(JSON.stringify(value), {
    status: 200,
    headers: { "content-type": "application/json" },
  });
}

test("connected client uses fixed same-origin GET endpoints only", async () => {
  const calls: Array<{ path: string; method: string | undefined; body: BodyInit | null | undefined }> = [];
  const client = new ConnectedTargetsClient({
    fetch: async (input, init) => {
      calls.push({ path: String(input), method: init?.method, body: init?.body });
      return response({
        schema: "offline-programming.connected-targets.v1",
        targets: [discovery()],
        issues: [],
      });
    },
  });
  const result = await client.discover();
  assert.equal(result.targets[0]?.profileId, "bench-relay");
  assert.deepEqual(calls, [{ path: CONNECTED_TARGETS_PATH, method: "GET", body: undefined }]);
});

test("connected client accepts unavailable read-only status and rejects physical capability drift", async () => {
  const target = discovery();
  let unsafe = false;
  const client = new ConnectedTargetsClient({
    fetch: async (input) => {
      if (String(input).endsWith("/status")) {
        return response({
          schema: "offline-programming.connected-target-status.v1",
          target,
          state: "unavailable",
          observed_at: "2026-07-30T10:00:00Z",
          status_available: false,
          plan_available: false,
          connected: false,
          reasons: ["observation_transport_not_installed"],
        });
      }
      return response({
        schema: "offline-programming.connected-capabilities.v1",
        manifest_backed_discovery: true,
        read_only_status: false,
        read_only_plan: false,
        identity_correlated: false,
        authority_observed: false,
        command_submission: unsafe,
        arm: false,
        weld_on: false,
        play: false,
        jog: false,
        teleop: false,
        raw_browser_transport: false,
        physical_execution: false,
        reasons: ["offline_programming_v1_physical_execution_disabled"],
      });
    },
  });
  const status = await client.status("bench-relay");
  assert.equal(status.connected, false);
  assert.equal(status.state, "unavailable");
  assert.equal((await client.capabilities("bench-relay")).commandSubmission, false);
  unsafe = true;
  await assert.rejects(() => client.capabilities("bench-relay"), /physical path/);
  await assert.rejects(() => client.status("../robot"), /identity/);
});

test("connected client fails closed on non-JSON HTTP errors", async () => {
  const client = new ConnectedTargetsClient({
    fetch: async () => new Response("Bad Gateway", {
      status: 502,
      headers: { "content-type": "text/plain" },
    }),
  });
  await assert.rejects(() => client.discover(), /connected read HTTP 502/);
});
