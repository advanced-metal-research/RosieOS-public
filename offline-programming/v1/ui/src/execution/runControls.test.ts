import assert from "node:assert/strict";
import test from "node:test";
import type { DenseTrajectoryRecord } from "../domain/project.ts";
import type { DenseStatus } from "./denseClient.ts";
import { deriveRunControls, planMotionBlocked, planMotionInstruction, PLAY_NEEDS_ARM, type RunControlsInput } from "./runControls.ts";

// The Machine panel and the Steam Deck pendant gate Load and Play on this one
// function; these pin the gates both of them rely on.
const record = {
  trajectoryDigest: "sha256:" + "a".repeat(64), planId: "plan", totalSampleCount: 100, totalDurationS: 1,
  createdAt: "2026-09-28T00:00:00Z", sourceProgramDigest: "sha256:" + "b".repeat(64), sourceProjectRevision: 4,
} as DenseTrajectoryRecord;

function statusOf(state: string, extra: { homeValid?: boolean; dryRun?: boolean; armed?: boolean; ready?: boolean } = {}): DenseStatus {
  return {
    backend: "rt_core", available: true, daemonError: "", armed: extra.armed ?? true,
    session: { id: "s", state, trajectoryDigest: record.trajectoryDigest, dryRun: extra.dryRun ?? false },
    rtCore: { ready: extra.ready ?? true, axes: [{ homeValid: extra.homeValid ?? true }], operatorFeedback: { axes: [] } },
  } as unknown as DenseStatus;
}

function input(overrides: Partial<RunControlsInput>): RunControlsInput {
  return {
    targetSelected: true, cellPending: false, inFlight: false, native: true, status: statusOf("idle"), record,
    projectRevision: 4, dryRun: false, daemonReachable: true, lastRequestError: null, planStartActive: false, ...overrides,
  };
}

test("Load waits for a current plan, fresh status and Home", () => {
  assert.equal(deriveRunControls(input({})).load.disabled, false);
  const stale = deriveRunControls(input({ projectRevision: 5 }));
  assert.equal(stale.load.disabled, true);
  assert.equal(stale.load.title, "The program changed since this was planned; plan it again");
  const silent = deriveRunControls(input({ daemonReachable: false }));
  assert.equal(silent.loadBlocker, "Waiting for fresh machine status before loading.");
  assert.equal(silent.load.disabled, true);
  const unhomed = deriveRunControls(input({ status: statusOf("idle", { homeValid: false }) }));
  assert.equal(unhomed.homeRequired, true);
  assert.equal(unhomed.load.disabled, true);
  assert.equal(deriveRunControls(input({ record: null })).load.disabled, true);
});

test("Play needs the plan staged in the chosen run mode, armed and ready", () => {
  const loaded = deriveRunControls(input({ status: statusOf("loaded") }));
  assert.equal(loaded.loaded, true);
  assert.equal(loaded.play.disabled, false);
  assert.equal(deriveRunControls(input({ status: statusOf("idle") })).play.disabled, true);
  const otherMode = deriveRunControls(input({ status: statusOf("loaded", { dryRun: true }) }));
  assert.equal(otherMode.play.disabled, true);
  assert.equal(otherMode.load.text, "Reload to robot");
  const playing = deriveRunControls(input({ status: statusOf("playing") }));
  assert.equal(playing.live, true);
  assert.equal(playing.play.disabled, true);
  assert.equal(playing.load.disabled, true, "nothing loads over a running program");
  assert.equal(playing.halt.disabled, false);
});

test("A plan staged on a disarmed or unready robot says to arm, not to load again", () => {
  for (const status of [statusOf("loaded", { armed: false }), statusOf("loaded", { ready: false })]) {
    const controls = deriveRunControls(input({ status }));
    assert.equal(controls.play.disabled, true);
    assert.equal(controls.play.title, PLAY_NEEDS_ARM);
    assert.equal(controls.load.disabled, false, "Load stays offered while disarmed");
  }
  // Nothing staged keeps its own reason; so does a plan staged in the other run mode.
  assert.equal(deriveRunControls(input({ status: statusOf("idle", { armed: false }) })).play.title, "Load to robot first");
  assert.equal(deriveRunControls(input({ status: statusOf("loaded", { armed: false, dryRun: true }) })).play.title, "Reload to robot to apply the selected run mode");
});

test("Plan waits for this client's own jog or move to end, in one instruction per app", () => {
  assert.equal(planMotionBlocked({ holding: false, moving: false }, "pendant"), "");
  assert.equal(planMotionBlocked({ holding: true, moving: false }, "pendant"), "Release the sticks and wait for the robot to stop, then PLAN");
  assert.equal(planMotionBlocked({ holding: false, moving: true }, "desktop"), planMotionInstruction("desktop"));
  for (const device of ["pendant", "desktop"] as const) {
    assert.doesNotMatch(planMotionInstruction(device), /press STOP|disarm/i, "a finished jog needs no Stop or disarm before planning");
  }
});

test("Stop is offered whenever a machine is selected, whatever else is in flight", () => {
  assert.equal(deriveRunControls(input({ inFlight: true, status: null })).stop.disabled, false);
  assert.equal(deriveRunControls(input({ targetSelected: false })).stop.disabled, true);
});

test("A retained loaded snapshot cannot enable Play after native feedback becomes stale", () => {
  const stale = deriveRunControls(input({ status: statusOf("loaded"), daemonReachable: false }));
  assert.equal(stale.loaded, true, "the staged identity is retained, but is not fresh authority to play");
  assert.equal(stale.play.disabled, true);
  assert.equal(stale.play.title, "Waiting for fresh machine status before playing.");
  assert.equal(stale.stop.disabled, false);
  assert.equal(deriveRunControls(input({ status: statusOf("loaded"), daemonReachable: true })).play.disabled, false);
});
