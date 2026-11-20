import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import {
  connectedPlanIsFresh,
  connectedStatusIsFresh,
  initialConnectedStreamState,
  normalizeConnectedPlan,
  normalizeConnectedStatus,
  reduceConnectedStreams,
} from "./connectedStreams.ts";

const nowMs = Date.parse("2026-07-30T08:00:00.000Z");

test("connected normalized streams preserve unknown physical output facts", () => {
  const status = normalizeConnectedStatus(statusWire(1));
  assert.equal(connectedStatusIsFresh(status, nowMs), true);
  assert.equal(status.output.armRequested.known, true);
  assert.equal(status.output.actualServoArmed.known, false);
  assert.equal(status.output.ethercatEnabled.known, false);
  assert.equal(status.output.motionOutputEnabled.known, false);
  assert.equal(status.output.weldOutputEnabled.known, false);
  assert.equal(status.authority.known, true);

  const plan = normalizeConnectedPlan(planWire(1));
  assert.equal(connectedPlanIsFresh(plan, nowMs), true);
  assert.equal(plan.identityCorrelated, true);
  assert.equal(plan.identity.planId, "plan-42");
  assert.equal(plan.safety.collision.clear, true);
});

test("connected stream reducer retains latest only and rejects stale sequence", () => {
  let state = initialConnectedStreamState();
  state = reduceConnectedStreams(state, { type: "status", value: statusWire(1) });
  state = reduceConnectedStreams(state, { type: "status", value: statusWire(2) });
  assert.equal(state.status?.sequence, 2);
  assert.equal(state.statusSequence, 2);
  state = reduceConnectedStreams(state, { type: "status", value: statusWire(1) });
  assert.equal(state.status?.sequence, 2);
  assert.deepEqual(state.blockedBy, ["status_sequence_stale_or_repeated"]);

  state = reduceConnectedStreams(state, { type: "plan", value: planWire(1) });
  assert.equal(state.plan?.sequence, 1);
  state = reduceConnectedStreams(state, { type: "plan", value: { schema: "wrong" } });
  assert.equal(state.plan?.sequence, 1);
  assert.deepEqual(state.blockedBy, ["connected_stream_frame_invalid"]);

  state = reduceConnectedStreams(state, { type: "disconnect" });
  assert.equal(state.status, null);
  assert.equal(state.plan, null);
  assert.deepEqual(state.blockedBy, ["connected_target_disconnected"]);
});

test("connected stream freshness and reason count fail closed", () => {
  const stale = normalizeConnectedStatus({
    ...statusWire(1),
    fresh_until: "2026-07-30T07:59:59.000Z",
  });
  assert.equal(connectedStatusIsFresh(stale, nowMs), false);

  const tooMany = {
    ...planWire(1),
    correlation_reasons: Array.from({ length: 33 }, () => "reason"),
  };
  assert.throws(
    () => normalizeConnectedPlan(tooMany),
    /reasons exceed bound/,
  );
});

test("connected browser modules contain no direct transport or send path", async () => {
  const sources = await Promise.all([
    readFile(new URL("./connectedDiscovery.ts", import.meta.url), "utf8"),
    readFile(new URL("./connectedStreams.ts", import.meta.url), "utf8"),
    readFile(new URL("../execution/gateway.ts", import.meta.url), "utf8"),
  ]);
  for (const source of sources) {
    assert.equal(/\bnew\s+WebSocket\b/.test(source), false);
    assert.equal(/\.send\s*\(/.test(source), false);
    assert.equal(/\bfetch\s*\(/.test(source), false);
    assert.equal(/\bnats:\/\//.test(source), false);
  }
});

export function statusWire(sequence: number): Record<string, unknown> {
  return {
    schema: "offline-programming.connected-status.v1",
    target: targetWire(),
    source_peer: "bench-relay",
    source_subject: "robot/v4/motion-server.bench-relay.status",
    sequence,
    received_at: "2026-07-30T08:00:00Z",
    source_updated_at: "2026-07-30T08:00:00Z",
    fresh_until: "2026-07-30T08:00:05Z",
    connected: true,
    identity_correlated: true,
    service_name: "robot-v4-cartesiand",
    service_state: "running",
    plan: {
      preloaded: fact(true, true),
      loaded_program_available: fact(true, true),
      loaded_program_playable: fact(true, true),
      server_validated: fact(true, true),
      play_authorized: fact(true, true),
      playback_active: fact(true, false),
      loaded_program_revision: 42,
      plan_revision: 42,
      plan_session_id: "plan-42",
    },
    output: {
      arm_requested: fact(true, false),
      current_zero_hold: fact(true, true),
      stale_zero_hold: fact(true, false),
      actual_servo_armed: fact(false, false),
      ethercat_enabled: fact(false, false),
      motion_output_enabled: fact(false, false),
      weld_output_enabled: fact(false, false),
    },
    authority: {
      known: true,
      controller_id: "steamdeck:steamdeck-primary",
      controller_boot_id: "boot-1",
      lease_id: "motion-server-abcd-0009",
      fence_epoch: 9,
      campaign_generation: 3,
      lease_fresh: true,
      transition_pending: false,
      expires_in: 1_500_000_000,
      fresh_until: "2026-07-30T08:00:01.5Z",
    },
    correlation_reasons: [],
  };
}

export function planWire(sequence: number): Record<string, unknown> {
  return {
    schema: "offline-programming.connected-plan.v1",
    target: targetWire(),
    source_peer: "bench-relay",
    source_subject: "robot/v4/motion-server.bench-relay.plan",
    sequence,
    received_at: "2026-07-30T08:00:00Z",
    fresh_until: "2026-07-30T08:00:05Z",
    identity: {
      plan_id: "plan-42",
      plan_revision: 42,
      program_id: "program-7",
      program_revision: 7,
      program_hash: "fnv1a64:1234",
      robot_model: "rosie_1400_v3",
    },
    planner_owner: "motion-server",
    planner_backend: "tesseract",
    validation_state: "valid",
    accepted: fact(true, true),
    play_enabled: fact(true, true),
    runtime_preload_enabled: fact(true, true),
    sample_count: 128,
    safety: {
      collision: evidence("tesseract_discrete_contact_checked"),
      singularity: evidence("jacobian_svd_checked"),
      limits: evidence("clear"),
      runtime: evidence("clear"),
    },
    identity_correlated: true,
    correlation_reasons: [],
  };
}

function targetWire(): Record<string, unknown> {
  return {
    profile_id: "bench-relay",
    manifest_ref: "rt-core/config/cells/bench-relay.json",
    cell_id: "bench-relay",
  };
}

function fact(known: boolean, value: boolean): Record<string, unknown> {
  return { known, value };
}

function evidence(state: string): Record<string, unknown> {
  return { available: true, clear: true, state, point_count: 0 };
}
