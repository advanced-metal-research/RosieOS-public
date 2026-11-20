import assert from "node:assert/strict";
import test from "node:test";

import {
  TARGET_DESCRIPTOR_SCHEMA,
  type TargetDescriptor,
} from "./contracts.ts";
import {
  initialTargetState,
  reduceTargetState,
  targetContextIsFresh,
} from "./stateMachine.ts";

const now = Date.UTC(2026, 6, 30, 10, 0, 0);

function simulatorDescriptor(): TargetDescriptor {
  return {
    schema: TARGET_DESCRIPTOR_SCHEMA,
    profileId: "local-simulator",
    label: "Tesseract",
    kind: "local_simulator",
    manifestRef: null,
    manifestSchema: null,
    cellId: null,
    axes: [{
      index: 0,
      name: "J1",
      urdfJoint: "J1",
      kind: "arm",
      gearRatio: 100,
      sign: 1,
      cartesianParticipates: true,
    }],
    capabilities: {
      monitoring: true,
      planning: true,
      simulation: true,
      realExecution: false,
      teleop: false,
      rawBrowserTransport: false,
    },
  };
}

function realDescriptor(profileId = "bench-relay"): TargetDescriptor {
  return {
    ...simulatorDescriptor(),
    profileId,
    label: profileId,
    kind: "real_cell",
    manifestRef: `rt-core/config/cells/${profileId}.json`,
    manifestSchema: "robot_v4_deployment_manifest_v1",
    cellId: profileId,
    capabilities: {
      monitoring: true,
      planning: true,
      simulation: false,
      realExecution: false,
      teleop: false,
      rawBrowserTransport: false,
    },
  };
}

test("local simulator selection requires fresh no-output proof", () => {
  const initial = initialTargetState(now);
  const unsafe = reduceTargetState(initial, {
    type: "select_local_simulator",
    atMs: now + 100,
    target: simulatorDescriptor(),
    proof: {
      profileId: "local-simulator",
      statusSequence: 2,
      observedAtMs: now,
      freshUntilMs: now + 1_000,
      noRealMovement: false,
      ethercatEnabled: false,
      motionOutputEnabled: false,
    },
  });
  assert.equal(unsafe.mode, "offline_preview");
  assert.equal(unsafe.phase, "blocked");
  assert.deepEqual(unsafe.blockedBy, ["local_simulator_safety_proof_invalid"]);
});

test("local simulator tolerates bounded host clock skew and rejects excess", () => {
  const target = simulatorDescriptor();
  const proof = {
    profileId: "local-simulator",
    statusSequence: 3,
    observedAtMs: now + 250,
    freshUntilMs: now + 1_000,
    noRealMovement: true,
    ethercatEnabled: false,
    motionOutputEnabled: false,
  };
  const accepted = reduceTargetState(initialTargetState(now), {
    type: "select_local_simulator",
    atMs: now,
    target,
    proof,
  });
  assert.equal(accepted.mode, "local_simulator");
  assert.equal(accepted.phase, "simulator_ready");

  const rejected = reduceTargetState(initialTargetState(now), {
    type: "select_local_simulator",
    atMs: now,
    target,
    proof: {
      ...proof,
      observedAtMs: now + 1_001,
      freshUntilMs: now + 2_000,
    },
  });
  assert.equal(rejected.mode, "offline_preview");
  assert.equal(rejected.phase, "blocked");
});

test("local simulator can remain explicitly selected while typed unavailable", () => {
  const initial = initialTargetState(now);
  const target = simulatorDescriptor();
  const blocked = reduceTargetState(initial, {
    type: "select_local_simulator_unavailable",
    atMs: now + 1,
    target,
    reason: "collision_runtime_unavailable",
  });
  assert.equal(blocked.mode, "local_simulator");
  assert.equal(blocked.phase, "blocked");
  assert.equal(blocked.target.profileId, target.profileId);
  assert.deepEqual(blocked.blockedBy, ["collision_runtime_unavailable"]);
});

test("real target switch and mismatched status fail closed", () => {
  let state = reduceTargetState(initialTargetState(now), {
    type: "select_real_target",
    atMs: now + 100,
    target: realDescriptor(),
  });
  assert.equal(state.phase, "connecting");
  state = reduceTargetState(state, {
    type: "observe_real_target",
    atMs: now + 200,
    observation: {
      profileId: "other-relay",
      manifestRef: "rt-core/config/cells/bench-relay.json",
      robotModel: "rosie_1400_v3",
      statusSequence: 5,
      observedAtMs: now + 150,
      freshUntilMs: now + 1_000,
      connected: true,
    },
  });
  assert.equal(state.phase, "blocked");
  assert.equal(targetContextIsFresh(state, now + 250), false);

  const switched = reduceTargetState(state, {
    type: "select_real_target",
    atMs: now + 300,
    target: realDescriptor("other-relay"),
  });
  assert.equal(switched.phase, "connecting");
  assert.equal(switched.observation, null);
});
