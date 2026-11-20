import assert from "node:assert/strict";
import test from "node:test";
import {
  CONNECTED_TARGET_DISCOVERY_SCHEMA,
  normalizeConnectedTargetDiscovery,
} from "./connectedDiscovery.ts";

test("connected discovery normalizes safe Go wire descriptor", () => {
  const discovery = normalizeConnectedTargetDiscovery(discoveryWire());
  assert.equal(discovery.schema, CONNECTED_TARGET_DISCOVERY_SCHEMA);
  assert.equal(discovery.profileId, "bench-relay");
  assert.equal(discovery.peerStaleAfterS, 120);
  assert.equal(discovery.robotModelBound, false);
  assert.equal(discovery.services.length, 2);
  assert.equal(discovery.target.capabilities.monitoring, true);
  assert.equal(discovery.target.capabilities.realExecution, false);
  assert.equal(discovery.target.capabilities.teleop, false);
  assert.equal(discovery.target.capabilities.rawBrowserTransport, false);
  assert.equal(JSON.stringify(discovery).includes("nats://"), false);
  assert.equal(JSON.stringify(discovery).includes("command_subject"), false);
});

test("connected discovery accepts a cell that publishes no axis gear ratio and rejects a non-positive one", () => {
  // The server's descriptor marks gear_ratio optional (omitempty): the example-relay and bench manifests
  // omit it, and the read-only connected-targets list must still render those cells.
  const wire = discoveryWire();
  const target = wire.target as Record<string, unknown>;
  const axis = (target.axes as Record<string, unknown>[])[0]!;
  delete axis.gear_ratio;
  const discovery = normalizeConnectedTargetDiscovery(wire);
  assert.equal(discovery.target.axes[0]!.gearRatio, null);
  assert.equal(discovery.target.axes[0]!.name, "J1");
  for (const bad of [0, -100, Number.NaN, "100"]) {
    const rejected = discoveryWire();
    ((rejected.target as Record<string, unknown>).axes as Record<string, unknown>[])[0]!.gear_ratio = bad;
    assert.throws(() => normalizeConnectedTargetDiscovery(rejected), /positive finite number required/);
  }
});

test("connected discovery rejects route injection and physical capabilities", () => {
  const injected = { ...discoveryWire(), binding: { nats_url: "nats://bench-relay:4222" } };
  assert.throws(
    () => normalizeConnectedTargetDiscovery(injected),
    /unsupported field binding/,
  );

  const physical = discoveryWire();
  const target = physical.target as Record<string, unknown>;
  target.capabilities = {
    ...(target.capabilities as Record<string, unknown>),
    real_execution: true,
  };
  assert.throws(
    () => normalizeConnectedTargetDiscovery(physical),
    /enables an unavailable physical capability/,
  );
});

test("connected discovery rejects ambiguous services and unbounded freshness", () => {
  const ambiguous = discoveryWire();
  ambiguous.services = [
    service("motion-server", true),
    service("motion-server", true),
  ];
  assert.throws(
    () => normalizeConnectedTargetDiscovery(ambiguous),
    /service roles are ambiguous/,
  );
  const unbounded = discoveryWire();
  unbounded.peer_stale_after_s = 301;
  assert.throws(
    () => normalizeConnectedTargetDiscovery(unbounded),
    /freshness is out of bounds/,
  );
});

function discoveryWire(): Record<string, unknown> {
  return {
    schema: "offline-programming.connected-target-discovery.v1",
    profile_id: "bench-relay",
    manifest_ref: "rt-core/config/cells/bench-relay.json",
    manifest_schema: "robot_v4_deployment_manifest_v1",
    cell_id: "bench-relay",
    peer_stale_after_s: 120,
    robot_model_bound: false,
    services: [
      service("motion-server", true),
      service("realtime-core", false),
    ],
    target: {
      schema: "offline-programming.target-descriptor.v1",
      profile_id: "bench-relay",
      label: "bench-relay",
      kind: "real_cell",
      manifest_ref: "rt-core/config/cells/bench-relay.json",
      manifest_schema: "robot_v4_deployment_manifest_v1",
      cell_id: "bench-relay",
      axes: [{
        index: 0,
        name: "J1",
        urdf_joint: "J1",
        kind: "arm",
        gear_ratio: 100,
        sign: -1,
        cartesian_participates: true,
      }],
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

function service(role: string, planStream: boolean): Record<string, unknown> {
  return {
    role,
    status_stream: true,
    plan_stream: planStream,
    read_only: true,
    server_mediated: true,
  };
}
