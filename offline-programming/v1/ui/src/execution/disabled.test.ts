import assert from "node:assert/strict";
import test from "node:test";

import {
  DisabledTeleopClient,
  TELEOP_CAPABILITY,
  TELEOP_DISABLED,
} from "./teleop.ts";

test("future tele-op boundary remains disabled and has no raw browser transport", async () => {
  assert.equal(TELEOP_CAPABILITY.enabled, false);
  assert.equal(TELEOP_CAPABILITY.serverOwned, true);
  assert.equal(TELEOP_CAPABILITY.rawBrowserTransport, false);
  const client = new DisabledTeleopClient();
  await assert.rejects(
    client.requestSession({
      targetProfileId: "local-simulator",
      requestedAxisCount: 9,
      deadmanRequired: true,
      releaseZeroRequired: true,
      maximumInputAgeMs: 100,
      maximumCommandRateHz: 50,
      maximumAxisVelocityRadS: Array(9).fill(0.1),
    }),
    new RegExp(TELEOP_DISABLED),
  );
});
