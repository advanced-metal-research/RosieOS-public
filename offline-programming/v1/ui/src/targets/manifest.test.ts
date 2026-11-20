import assert from "node:assert/strict";
import test from "node:test";

import { descriptorFromRobotV4Manifest } from "./manifest.ts";

function manifestFixture(): Record<string, unknown> {
  return {
    schema_version: "robot_v4_deployment_manifest_v1",
    name: "jetson-test",
    robot_command_subject: "robot/v4/robot.jetson-test.command",
    runtime_connections: {
      motion_server_udp_host: "private.example",
      motion_server_nats_url: "nats://private.example:4222",
    },
    robot_axes: {
      axes: [
        {
          index: 0,
          name: "J1",
          urdf_joint: "J1",
          kind: "arm",
          gear_ratio: 100,
          sign: -1,
          cartesian_participates: true,
        },
        {
          index: 1,
          name: "J2",
          urdf_joint: "J2",
          kind: "arm",
          gear_ratio: 100,
          sign: 1,
          cartesian_participates: true,
        },
      ],
    },
    nodes: [
      {
        node: "motion-server",
        peer: "jetson-test",
        role: "motion-server",
        host: "jetson-test",
        cold_path_subjects: [
          "robot/v4/motion-server.jetson-test.status",
          "robot/v4/motion-server.jetson-test.plan",
        ],
      },
      {
        node: "realtime-core",
        peer: "revpi-test",
        role: "realtime-core",
        host: "RevPiTest",
        cold_path_subjects: [
          "robot/v4/rtcore.RevPiTest.status",
          "robot/v4/robot.jetson-test.command",
        ],
      },
    ],
  };
}

test("manifest descriptor exposes identity and axes without browser transports", () => {
  const descriptor = descriptorFromRobotV4Manifest(
    "rt-core/config/cells/jetson-test.json",
    manifestFixture(),
  );
  assert.equal(descriptor.profileId, "jetson-test");
  assert.equal(descriptor.axes.length, 2);
  assert.equal(descriptor.capabilities.realExecution, false);
  assert.equal(descriptor.capabilities.teleop, false);
  assert.equal(descriptor.capabilities.rawBrowserTransport, false);
  const encoded = JSON.stringify(descriptor).toLowerCase();
  assert.doesNotMatch(encoded, /nats:|udp|command_subject|private\.example/);
});

test("manifest descriptor rejects mismatched command and axis identities", () => {
  const commandMismatch = manifestFixture();
  commandMismatch.robot_command_subject = "robot/v4/robot.other.command";
  assert.throws(
    () => descriptorFromRobotV4Manifest("fixture.json", commandMismatch),
    /does not match/,
  );

  const axisMismatch = manifestFixture();
  const robotAxes = axisMismatch.robot_axes as { axes: Array<Record<string, unknown>> };
  robotAxes.axes[1]!.name = "J3";
  assert.throws(
    () => descriptorFromRobotV4Manifest("fixture.json", axisMismatch),
    /canonical J1-J9 order/,
  );
});
